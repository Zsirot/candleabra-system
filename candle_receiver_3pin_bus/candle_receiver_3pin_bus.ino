/*
 * candle_receiver_3pin_bus.ino
 *
 * Lives inside a candelabra. Drives 3 pixels.
 * Listens for ESP-NOW packets and renders effects LOCALLY —
 * the packet only says WHAT to do, never frame-by-frame values.
 *
 * Set FIXTURE_ID to 1 on one candelabra and 2 on the other,
 * then flash the same file to both.
 *
 * ADDRESSING
 *   F0            both candelabras, all candles
 *   F1  / F2      fixture A / B, all three candles
 *   F11 F12 F13   fixture A, candle 1 / 2 / 3
 *   F21 F22 F23   fixture B, candle 1 / 2 / 3
 *
 * Every candle holds its own complete state, so a group command and a
 * per-candle command simply overwrite whatever they address. Last
 * received wins — there is no priority and no modal state.
 *
 * Board: ESP32 Dev Module.  Requires esp32 core 3.x.
 * Library: NeoPixelBus by Makuna.
 */

#include <NeoPixelBus.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ---------------------------------------------------------------- config

#define FIXTURE_ID   1        // <-- 1 on candelabra A, 2 on candelabra B
// One pixel per candle, each on its own GPIO. No chaining, so a dead
// pixel never takes the others down with it.
#define PIN_1        4
#define PIN_2        5
#define PIN_3       18
#define NUM_PIXELS   3        // candles per fixture
#define USE_RGBW     1        // 1 = SK6812 RGBW pixels. 0 = WS2812B bench strip.
#define WIFI_CHANNEL 1        // pin the channel; floating channels cause jitter

// Many USB power banks shut off when draw stays under ~50-100mA. During a
// blackout the rig sits right around that line. This briefly pulses the
// pixels to a level the bank notices but the audience does not.
// Currently OFF. Turn back on only if the bank actually cuts out during a
// long blackout; if it does, drop KEEPALIVE_LEVEL to ~25 so it stays subtle.
#define KEEPALIVE_MS        0   // how often to pulse (0 = disabled)
#define KEEPALIVE_LEN      90   // how long the pulse lasts, ms
#define KEEPALIVE_LEVEL   110   // 0-255 per channel during the pulse

#define FAILSAFE_MS  3000     // no packets for this long -> drop to the idle look

// What the idle look is, when the transmitter goes quiet.
#define FAILSAFE_MASTER 255
#define FAILSAFE_SPEED  170
#define FAILSAFE_DEPTH  220
#define FAILSAFE_WARMTH 0

// --- pixel type. Each bus instance needs its OWN RMT channel.
#if USE_RGBW
  #define PIXFEATURE  NeoGrbwFeature
  #define PIXMETHOD0  NeoEsp32Rmt0Sk6812Method
  #define PIXMETHOD1  NeoEsp32Rmt1Sk6812Method
  #define PIXMETHOD2  NeoEsp32Rmt2Sk6812Method
  typedef RgbwColor PixColor;
#else
  #define PIXFEATURE  NeoGrbFeature
  #define PIXMETHOD0  NeoEsp32Rmt0Ws2812xMethod
  #define PIXMETHOD1  NeoEsp32Rmt1Ws2812xMethod
  #define PIXMETHOD2  NeoEsp32Rmt2Ws2812xMethod
  typedef RgbColor PixColor;
#endif

// ---------------------------------------------------------------- protocol

// Must match the transmitter byte for byte.
typedef struct __attribute__((packed)) {
  uint8_t magic;    // 0xC1, cheap sanity check
  uint8_t fixture;  // see ADDRESSING above
  uint8_t preset;
  uint8_t master;
  uint8_t speed;
  uint8_t depth;
  uint8_t warmth;
  uint8_t fade;     // crossfade time in units of 10ms (0 = snap)
  uint8_t hue;
  uint8_t sat;
  uint8_t boost;    // 0 = flame as always, 255 = all four dies at full
} CandlePacket;

#define MAGIC 0xC1

enum Preset {
  P_BLACKOUT = 0,
  P_STEADY,
  P_FLICKER,
  P_GUST,
  P_EMBER,
  P_PULSE,
  P_STROBE,
  P_COUNT
};

// ---------------------------------------------------------------- state

// Everything a single candle needs. Group commands write the same values
// into all three; per-candle commands touch only one.
struct Candle {
  uint8_t preset     = P_FLICKER;
  uint8_t prevPreset = P_FLICKER;
  uint8_t master     = FAILSAFE_MASTER;
  uint8_t speed      = FAILSAFE_SPEED;
  uint8_t depth      = FAILSAFE_DEPTH;
  uint8_t warmth     = FAILSAFE_WARMTH;
  uint8_t hue        = 0;
  uint8_t sat        = 0;
  uint8_t boost      = 0;

  unsigned long fadeStart = 0;
  unsigned long fadeLen   = 0;   // 0 = not fading

  // flame state, independent per candle so they never move in lockstep
  float         level     = 0.8;
  float         target    = 0.8;
  unsigned long gustUntil = 0;
  float         gustDepth = 0;
  unsigned long lastStep  = 0;
  unsigned long phaseOff  = 0;   // desync pulse slightly between candles
};

Candle cd[NUM_PIXELS];

// Packets arrive on a callback task. Buffer them rather than mutating
// candle state from two contexts at once.
#define PKT_QUEUE 8
volatile CandlePacket  pktQueue[PKT_QUEUE];
volatile uint8_t       qHead = 0, qTail = 0;
volatile unsigned long lastRx = 0;

bool inFailsafe = false;

// ---------------------------------------------------------------- pixels
//
// These sit below the type definitions on purpose. Arduino inserts its
// generated function prototypes immediately before the FIRST function
// definition in the sketch, so any function declared above struct Candle
// or CandlePacket produces a prototype that references an unknown type.
// Keep every type above every function and the generator stays happy.

NeoPixelBus<PIXFEATURE, PIXMETHOD0> px1(1, PIN_1);
NeoPixelBus<PIXFEATURE, PIXMETHOD1> px2(1, PIN_2);
NeoPixelBus<PIXFEATURE, PIXMETHOD2> px3(1, PIN_3);

// The three objects are different C++ types, so no array of pointers.
static inline void setCandle(int i, const PixColor& c) {
  switch (i) {
    case 0: px1.SetPixelColor(0, c); break;
    case 1: px2.SetPixelColor(0, c); break;
    case 2: px3.SetPixelColor(0, c); break;
  }
}
static inline void showCandles() {
  px1.Show();
  px2.Show();
  px3.Show();
}

// ---------------------------------------------------------------- color

// LEDs are driven linearly but the eye is not, so a straight multiply
// makes colour drift as things dim. Linearise first, then scale.
static inline float gam(float srgb255) {
  return pow(srgb255 / 255.0, 2.2);
}

// hue 0-1 -> full-saturation RGB, 0-255 each. Standard 6-sector HSV.
static void hueToRGB(float h, float& r, float& g, float& b) {
  h = fmodf(h, 1.0f) * 6.0f;
  int   i = (int)h;
  float f = h - i;
  float q = 1.0f - f;
  switch (i) {
    case 0: r = 1;  g = f;  b = 0;  break;
    case 1: r = q;  g = 1;  b = 0;  break;
    case 2: r = 0;  g = 1;  b = f;  break;
    case 3: r = 0;  g = q;  b = 1;  break;
    case 4: r = f;  g = 0;  b = 1;  break;
    default:r = 1;  g = 0;  b = q;  break;
  }
  r *= 255; g *= 255; b *= 255;
}

/*
 * Candle flame sits around 1600-2400K, far warmer than most "warm white".
 *   w = 0.0  ->  ~2400K, pale gold
 *   w = 1.0  ->  ~1600K, deep orange, almost no blue
 */
PixColor flameColor(const Candle& c, float bright, float w) {
  bright = constrain(bright, 0.0f, 1.0f);
  w      = constrain(w,      0.0f, 1.0f);

  // --- blackbody flame, the default
  float r = 255.0;
  float g = 147.0 - (w * 62.0);    // 147 -> 85
  float b =  41.0 - (w * 41.0);    //  41 -> 0

  // The green die in a WS2812B/SK6812 is disproportionately bright for its
  // nominal value. Without this trim, warm tones read yellow-green.
  // 0.80 was the original, most accurate value. Raised for more raw output —
  // drop it back toward 0.80 if the flame starts looking yellow-green.
  g *= 0.88;

  // --- blend toward a saturated hue as sat comes up.
  float s = c.sat / 255.0;
  if (s > 0.001) {
    float hr, hg, hb;
    hueToRGB(c.hue / 255.0, hr, hg, hb);
    hg *= 0.88;                       // same green trim
    r = r * (1.0 - s) + hr * s;
    g = g * (1.0 - s) + hg * s;
    b = b * (1.0 - s) + hb * s;
  }

  float scl = bright * (c.master / 255.0);

  float R = gam(r) * scl;
  float G = gam(g) * scl;
  float B = gam(b) * scl;

#if USE_RGBW
  // White die carries the body of a flame, but must get out of the way for
  // saturated colour or everything washes out pastel.
  float Wch     = gam(255.0 * (1.0 - w * 0.30)) * scl * (1.0 - s);
  float rgbGain = 0.35 + s * 0.65;

  float Rout = R * rgbGain;
  float Gout = G * rgbGain;
  float Bout = B * rgbGain;

  /*
   * BOOST. Nothing in the normal colour path ever gets all four dies up at
   * once: rgbGain holds RGB down whenever saturation is low, and the white
   * die is switched off as saturation rises. So the brightest thing the
   * fixture could previously make was the flame, with most of the RGB
   * output unused.
   *
   * boost blends from whatever colour was computed toward all four channels
   * at full. 0 leaves everything exactly as it was, 255 is maximum output.
   */
  if (c.boost > 0) {
    float k = (c.boost / 255.0) * bright * (c.master / 255.0);
    Rout += (scl - Rout) * k;
    Gout += (scl - Gout) * k;
    Bout += (scl - Bout) * k;
    Wch  += (scl - Wch ) * k;
  }

  return PixColor((uint8_t)(Rout * 255),
                  (uint8_t)(Gout * 255),
                  (uint8_t)(Bout * 255),
                  (uint8_t)(Wch  * 255));
#else
  float Rout = R, Gout = G, Bout = B;
  if (c.boost > 0) {
    float k = (c.boost / 255.0) * bright * (c.master / 255.0);
    Rout += (scl - Rout) * k;
    Gout += (scl - Gout) * k;
    Bout += (scl - Bout) * k;
  }
  return PixColor((uint8_t)(Rout*255), (uint8_t)(Gout*255), (uint8_t)(Bout*255));
#endif
}

// ---------------------------------------------------------------- effects
//
// Each returns brightness 0..1 for ONE candle. Colour is applied
// afterwards so crossfades blend cleanly.

float fxFlicker(Candle& c, bool gusts) {
  unsigned long now = millis();

  // Advance the random walk on a fixed 16ms tick, independent of loop speed.
  if (now - c.lastStep < 16) return constrain(c.level, 0.03f, 1.0f);
  c.lastStep = now;

  int   chance = 2 + (c.speed / 24);            // 2 - 12
  float ease   = 0.05 + (c.speed / 255.0) * 0.25;
  float range  = 0.15 + (c.depth / 255.0) * 0.55;

  if (random(100) < chance) {
    c.target = (1.0 - range) + (random(1000) / 1000.0) * range;
  }
  c.level += (c.target - c.level) * ease;

  float b = c.level;

  if (gusts) {
    if (now > c.gustUntil && random(1000) < 4) {
      c.gustUntil = now + 700 + random(700);
      c.gustDepth = (c.depth / 255.0) * 0.7;
    }
    if (now < c.gustUntil) {
      float remaining = (c.gustUntil - now) / 1000.0;
      b -= c.gustDepth * sin(remaining * PI);
    }
  }
  return constrain(b, 0.03f, 1.0f);
}

float fxEmber(Candle& c) {
  // Low, slow, barely alive. Deliberately ignores speed and depth.
  unsigned long now = millis();
  if (now - c.lastStep < 16) return constrain(c.level, 0.02f, 0.4f);
  c.lastStep = now;

  if (random(100) < 2) c.target = 0.10 + (random(200) / 1000.0);
  c.level += (c.target - c.level) * 0.03;
  return constrain(c.level, 0.02f, 0.4f);
}

float fxPulse(Candle& c) {
  unsigned long period = 4000 - (c.speed * 12);   // 4s down to ~1s
  float phase = ((millis() + c.phaseOff) % period) / (float)period;
  float b = (sin(phase * TWO_PI - HALF_PI) + 1.0) / 2.0;
  float lo = 1.0 - (c.depth / 255.0);
  return lo + b * (1.0 - lo);
}

float fxStrobe(Candle& c) {
  // speed 0 -> 500ms period (2Hz), speed 255 -> 40ms period (25Hz)
  unsigned long period = 500 - (unsigned long)((c.speed / 255.0) * 460);
  if (period < 40) period = 40;

  // depth sets duty cycle: 8% = sharp stabs, 60% = chunky flashes
  float duty = 0.08 + (c.depth / 255.0) * 0.52;
  unsigned long onTime = (unsigned long)(period * duty);
  if (onTime < 3) onTime = 3;

  return ((millis() % period) < onTime) ? 1.0 : 0.0;
}

float renderCandle(Candle& c, uint8_t preset) {
  switch (preset) {
    case P_BLACKOUT: return 0.0;
    case P_STEADY:   return 1.0;
    case P_FLICKER:  return fxFlicker(c, false);
    case P_GUST:     return fxFlicker(c, true);
    case P_EMBER:    return fxEmber(c);
    case P_PULSE:    return fxPulse(c);
    case P_STROBE:   return fxStrobe(c);
    default:         return fxFlicker(c, false);
  }
}

// ---------------------------------------------------------------- routing

/*
 * Which of our candles does this fixture code address?
 * Returns a 3-bit mask, or 0 if the packet is not for this board.
 */
static uint8_t maskFor(uint8_t fixture) {
  if (fixture == 0) return 0b111;                     // everything
  if (fixture == FIXTURE_ID) return 0b111;            // this whole fixture

  uint8_t tens = fixture / 10;
  uint8_t ones = fixture % 10;
  if (tens == FIXTURE_ID && ones >= 1 && ones <= NUM_PIXELS) {
    return 1 << (ones - 1);                           // one candle
  }
  return 0;
}

static void applyPacket(const CandlePacket& p, uint8_t mask) {
  unsigned long now = millis();
  for (int i = 0; i < NUM_PIXELS; i++) {
    if (!(mask & (1 << i))) continue;
    Candle& c = cd[i];

    c.master = p.master;
    c.speed  = p.speed;
    c.depth  = p.depth;
    c.warmth = p.warmth;
    c.hue    = p.hue;
    c.sat    = p.sat;
    c.boost  = p.boost;

    if (p.preset != c.preset && p.preset < P_COUNT) {
      c.prevPreset = c.preset;
      c.preset     = p.preset;
      c.fadeLen    = (unsigned long)p.fade * 10;
      c.fadeStart  = now;
    }
  }
}

// ---------------------------------------------------------------- esp-now

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(CandlePacket)) return;

  CandlePacket p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != MAGIC) return;

  // Any valid packet proves the transmitter is alive, whoever it was
  // addressed to. Stamping before the fixture filter stops the other
  // fixture timing out into the idle look while you drive this one.
  lastRx = millis();

  if (maskFor(p.fixture) == 0) return;    // not for us

  uint8_t next = (qHead + 1) % PKT_QUEUE;
  if (next == qTail) return;              // queue full, drop this one
  memcpy((void*)&pktQueue[qHead], &p, sizeof(p));
  qHead = next;
}

// ---------------------------------------------------------------- setup

void setup() {
  Serial.begin(115200);
  delay(200);

  px1.Begin(); px1.Show();
  px2.Begin(); px2.Show();
  px3.Begin(); px3.Show();

  randomSeed(esp_random());
  for (int i = 0; i < NUM_PIXELS; i++) cd[i].phaseOff = i * 370;

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    return;
  }
  esp_now_register_recv_cb(onRecv);

  Serial.print("receiver ready, fixture ");
  Serial.println(FIXTURE_ID);
  Serial.print("responds to F0, F");
  Serial.print(FIXTURE_ID);
  Serial.print(", and F");
  Serial.print(FIXTURE_ID * 10 + 1);
  Serial.print("-F");
  Serial.println(FIXTURE_ID * 10 + NUM_PIXELS);
  Serial.print("my MAC: ");
  Serial.println(WiFi.macAddress());
}

// ---------------------------------------------------------------- loop

void loop() {
  unsigned long now = millis();

  // --- drain any commands that arrived
  while (qTail != qHead) {
    CandlePacket p;
    memcpy(&p, (const void*)&pktQueue[qTail], sizeof(p));
    qTail = (qTail + 1) % PKT_QUEUE;
    applyPacket(p, maskFor(p.fixture));
    inFailsafe = false;
  }

  // --- failsafe: lost the transmitter, fall back to a lively gusting flame
  if (lastRx != 0 && now - lastRx > FAILSAFE_MS && !inFailsafe) {
    inFailsafe = true;
    CandlePacket idle = { MAGIC, 0, P_GUST, FAILSAFE_MASTER, FAILSAFE_SPEED,
                          FAILSAFE_DEPTH, FAILSAFE_WARMTH, 100, 0, 0, 0 };
    applyPacket(idle, 0b111);
  }

  // --- render each candle from its own state
  for (int i = 0; i < NUM_PIXELS; i++) {
    Candle& c = cd[i];

    float b = renderCandle(c, c.preset);

    if (c.fadeLen > 0) {
      unsigned long elapsed = now - c.fadeStart;
      if (elapsed >= c.fadeLen) {
        c.fadeLen = 0;
      } else {
        float blend = (float)elapsed / (float)c.fadeLen;
        // renderCandle mutates flame state, so evaluate the outgoing look
        // on a scratch copy rather than advancing the random walk twice.
        Candle tmp = c;
        float prev = renderCandle(tmp, c.prevPreset);
        b = prev * (1.0 - blend) + b * blend;
      }
    }

    // dimmer reads redder, the way real flame does
    float w = (c.warmth / 255.0) * (1.0 - b * 0.35) + (1.0 - b) * 0.2;
    setCandle(i, flameColor(c, b, w));
  }

  // --- power bank keepalive, only when nothing is meaningfully lit
  if (KEEPALIVE_MS > 0) {
    bool dark = true;
    for (int i = 0; i < NUM_PIXELS; i++) {
      if (cd[i].preset != P_BLACKOUT && cd[i].master >= 12) dark = false;
    }
    if (dark && (now % KEEPALIVE_MS) < KEEPALIVE_LEN) {
#if USE_RGBW
      PixColor k(KEEPALIVE_LEVEL, 0, 0, 0);
#else
      PixColor k(KEEPALIVE_LEVEL, 0, 0);
#endif
      for (int i = 0; i < NUM_PIXELS; i++) setCandle(i, k);
    }
  }

  showCandles();
  delay(4);   // ~250fps, so fast strobes land cleanly
}
