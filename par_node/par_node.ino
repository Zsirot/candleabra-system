/*
 * par_node.ino
 *
 * ESP-NOW -> DMX512 node. Lives in a project box beside two battery pars
 * (DazzlingStage 4x18W RGBWA+UV) wired DMX IN / DMX OUT.
 *
 * The effect engine below is lifted from candle_receiver_3pin_bus.ino with
 * the numbers unchanged, so a par and a candle given the same packet move
 * the same way. Only the final render differs: DMX slots instead of pixels.
 *
 * ADDRESSING  — par groups are global, so a group can span both nodes.
 *   F40           all four pars
 *   F41 .. F44    one par, by global number
 *   F45           pars 1 and 2      (candelabra A's pair)
 *   F46           pars 3 and 4      (candelabra B's pair)
 *   F47           pars 1 and 4      (inside)
 *   F48           pars 2 and 3      (outside)
 *
 * TUNING OVER THE AIR. Fixture codes 90-95 carry a trim value in the master
 * field, 0-255 mapped to 0.0-1.0. They set a variable and light nothing, so
 * they can be sent mid-look without disturbing anything.
 *   F90 M<v>   whiteW      F93 M<v>   trimFlame
 *   F91 M<v>   whiteA      F94 M<v>   trimColour
 *   F92 M<v>   greenTrim   F95 M<v>   dimGamma
 * The transmitter forwards any code it is given, so this needs no change
 * there. The candle receivers never match a code whose tens digit is 9.
 *
 * F0 is deliberately NOT matched here. It addresses the candles only, so
 * MIDI channel 1 can drive both candelabras without dragging the pars along
 * — which is what makes link mode on the par channels mean anything.
 * F99 heartbeats match nothing, by design.
 *
 * WIRING
 *   GPIO 17 -> DI
 *   GPIO 16 -> DE and RE, jumpered together
 *   VIN     -> VCC      (MAX485 is a 5V part)
 *   GND     -> GND
 *   A -> XLR pin 3, B -> XLR pin 2, GND -> XLR pin 1
 *
 * PAR SETTINGS
 *   Chnd 10CH    CH1 dim, 2 R, 3 G, 4 B, 5 W, 6 A, 7 UV, 8 strobe, 9 mode, 10 speed
 *   A001 / A011  one address per par
 *   FAIL HOLd    hold the last look if DMX stops, don't black out
 *   2.4G OFF     keep its wireless receiver out of ESP-NOW's band
 *   IRCL OFF     no stray remote overriding a cue
 *   NODE NOR     not ECO
 * CH9 above 9 hands the fixture to its own auto modes. It is pinned at 0.
 *
 * Board: ESP32 Dev Module, esp32 core 3.x.
 * Library: esp_dmx by someweisguy, 4.x.
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_dmx.h>

// ---------------------------------------------------------------- config

/*
 * Which of the four pars in the rig this box drives, and in what order.
 * Global par numbers are 1-4, left to right across the stage.
 *   bench, all four on one node :  NUM_PARS 4,  PAR_INDEX { 1, 2, 3, 4 }
 *   node A                      :  NUM_PARS 2,  PAR_INDEX { 1, 2 }
 *   node B                      :  NUM_PARS 2,  PAR_INDEX { 3, 4 }
 */
#define NUM_PARS      4
static const uint8_t PAR_INDEX[NUM_PARS] = { 1, 2, 3, 4 };

#define NODE_ID       5        // label only, shows up in the serial banner
#define WIFI_CHANNEL  1        // must match the transmitter

#define PIN_TX       17
#define PIN_RTS      16        // DE + RE together
#define PIN_RX       21        // unused; RO is not connected

static const uint16_t PAR_ADDR[NUM_PARS] = { 1, 11, 21, 31 };   // A001 A011 A021 A031
#define CHANS_PER_PAR 10

// Bitmask with one bit per par, so widening NUM_PARS needs no other edits.
#define ALL_PARS ((1 << NUM_PARS) - 1)

#define FAILSAFE_MS     3000
#define FAILSAFE_MASTER  255
#define FAILSAFE_SPEED   170
#define FAILSAFE_DEPTH   220
#define FAILSAFE_WARMTH    0

// Same green trim as the candles. Drop toward 0.80 if warm tones read
// yellow-green on the par.
float greenTrim = 0.88;

/*
 * WARM WHITE SYNTHESIS.
 *
 * The candles use warm-white SK6812s — their W die is already around 2700K,
 * so the receiver can drive it hard and still look like a flame. The par's
 * white emitter is neutral or cool. Sending the candle's W value straight
 * across is why the par reads stark and bluish beside a candle: same number,
 * completely different colour temperature.
 *
 * So the par's white content is split across its white AND amber emitters,
 * which together approximate warm white. WHITE_W is how much of the cool die
 * to use, WHITE_A how much amber sits beside it.
 *
 *   too white / too cold  ->  lower WHITE_W, or raise WHITE_A
 *   too orange / too dim  ->  raise WHITE_W, or lower WHITE_A
 *
 * Set whiteW to 1.0 and whiteA to 0.0 to get the old behaviour back.
 *
 * These are VARIABLES, not constants, and can be tuned live over this node's
 * USB serial while a candle sits next to a par — see the command list at the
 * bottom of this file. Once they look right, paste the numbers back in here
 * so they survive a reflash.
 */
// Tuned by eye against a lit candelabra. The white die turned out to be far
// less dominant than expected — it wants a bit over half, not a fraction.
float whiteW = 0.621;    // cool white die share
float whiteA = 0.974;    // amber share

/*
 * OUTPUT TRIM, split in two.
 *
 * Four 18W emitters against twelve SK6812 pixels is not a fair fight — at the
 * same master the par buries the candelabra. But the two need different
 * amounts of help: the flame path drives the par's white and amber hard and
 * wants pulling right down, while a saturated colour is already held back by
 * rgbGain and mostly wants leaving alone.
 *
 * So the trim is interpolated by saturation. trimFlame applies at sat 0,
 * trimColour at sat 255, and anything between blends. Scales CH1 only —
 * colour ratios are untouched either way.
 */
float trimFlame  = 0.293;    // candle flame — the par is far louder than
                             // twelve pixels and needs pulling right down
float trimColour = 1.000;    // saturated palettes — left alone deliberately

/*
 * DIMMER CURVE.
 *
 * A pixel's low end is linear: ask for 4% and you get 4%. Most cheap DMX
 * fixtures are not — everything under about CH1=15 collapses to almost
 * nothing. That is why deep ember goes missing on a par while reading fine on
 * a candle: master 27 times ember's own 0.4 ceiling lands near CH1=10, which
 * is inside the dead zone.
 *
 * This applies a gamma to CH1 only. Below 1.0 the low end lifts while full
 * output stays put, so dim looks come back without anything else getting
 * brighter.
 *
 *   1.00  no change, raw linear
 *   0.47  tuned by eye — the par's low end was well and truly crushed
 *
 * Colour is untouched. This is purely the level channel.
 */
float dimGamma = 0.469;

// Channel offsets within a par's 10-slot personality
enum { CH_DIM = 0, CH_R, CH_G, CH_B, CH_W, CH_A, CH_UV, CH_STROBE, CH_MODE, CH_SPEED };

// ---------------------------------------------------------------- protocol

// Must match candle_transmitter.ino byte for byte.
typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t fixture;
  uint8_t preset;
  uint8_t master;
  uint8_t speed;
  uint8_t depth;
  uint8_t warmth;
  uint8_t fade;     // preset crossfade, units of 10ms
  uint8_t hue;
  uint8_t sat;
  uint8_t boost;    // 0 = flame as always, 255 = all emitters at full
} CandlePacket;

#define MAGIC 0xC1

enum Preset {
  P_BLACKOUT = 0, P_STEADY, P_FLICKER, P_GUST, P_EMBER, P_PULSE, P_STROBE, P_COUNT
};

// ---------------------------------------------------------------- state

// Field for field the candle's struct. Keeping them identical is what lets
// the effect functions below be a straight copy.
struct Par {
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
  unsigned long fadeLen   = 0;

  float         level     = 0.8;
  float         target    = 0.8;
  unsigned long gustUntil = 0;
  float         gustDepth = 0;
  unsigned long lastStep  = 0;
  unsigned long phaseOff  = 0;
};

Par pr[NUM_PARS];

#define PKT_QUEUE 8
volatile CandlePacket  pktQueue[PKT_QUEUE];
volatile uint8_t       qHead = 0, qTail = 0;
volatile unsigned long lastRx = 0;

bool inFailsafe = false;
volatile bool tuneDirty = false;   // an over-the-air trim arrived

dmx_port_t dmxPort = DMX_NUM_1;
uint8_t    dmxData[DMX_PACKET_SIZE];

// Only send the slots actually in use. A 21-slot frame refreshes far faster
// than a full 512, which is what lets a 25Hz strobe land cleanly.
static const int DMX_SLOTS = 1 + PAR_ADDR[NUM_PARS - 1] + CHANS_PER_PAR;

// ---------------------------------------------------------------- colour

static inline float gam(float srgb255) {
  return pow(srgb255 / 255.0, 2.2);
}

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
 * Same blackbody flame as the candles, rendered into one par's ten slots.
 *
 * The one deliberate departure: brightness rides CH1 rather than being baked
 * into the colour values. A pixel can dim by scaling RGB because it has the
 * whole 8-bit range to itself, but doing that here would leave a dim flame at
 * R=3 G=1 B=0 with almost no colour resolution left. Colour stays at full
 * scale, CH1 carries level. The ratios come out the same either way.
 */
static void flameDmx(const Par& c, float bright, float w, uint8_t* slot) {
  bright = constrain(bright, 0.0f, 1.0f);
  w      = constrain(w,      0.0f, 1.0f);

  float r = 255.0;
  float g = 147.0 - (w * 62.0);
  float b =  41.0 - (w * 41.0);
  g *= greenTrim;

  float s = c.sat / 255.0;
  if (s > 0.001) {
    float hr, hg, hb;
    hueToRGB(c.hue / 255.0, hr, hg, hb);
    hg *= greenTrim;
    r = r * (1.0 - s) + hr * s;
    g = g * (1.0 - s) + hg * s;
    b = b * (1.0 - s) + hb * s;
  }

  float R = gam(r);
  float G = gam(g);
  float B = gam(b);
  // The candle's single warm-white value, before it is split across two
  // emitters. Identical maths to the receiver up to this point.
  float Wraw = gam(255.0 * (1.0 - w * 0.30)) * (1.0 - s);

  float W = Wraw * whiteW;
  float A = Wraw * whiteA;
  float rgbGain = 0.35 + s * 0.65;

  float scl = bright * (c.master / 255.0);

  float Rout = R * rgbGain;
  float Gout = G * rgbGain;
  float Bout = B * rgbGain;

  /*
   * BOOST. Same blend as the candles: push every emitter toward full. On a
   * pixel the target is `scl`, because brightness is baked into the colour
   * there. Here colour is normalised and brightness rides CH1, so the target
   * is 1.0 — but k still carries scl, so boost stays level-dependent exactly
   * as it does on the candles.
   */
  if (c.boost > 0) {
    float k = (c.boost / 255.0) * scl;
    Rout += (1.0 - Rout) * k;
    Gout += (1.0 - Gout) * k;
    Bout += (1.0 - Bout) * k;
    W    += (1.0 - W   ) * k;
    A    += (1.0 - A   ) * k;   // amber included, or boost would go cold
  }

  float trim  = trimFlame + (trimColour - trimFlame) * s;
  float level = constrain(scl * trim, 0.0f, 1.0f);
  if (dimGamma != 1.0f && level > 0.0f) level = powf(level, dimGamma);
  slot[CH_DIM]    = (uint8_t)constrain(level * 255.0, 0.0f, 255.0f);
  slot[CH_R]      = (uint8_t)constrain(Rout * 255.0, 0.0f, 255.0f);
  slot[CH_G]      = (uint8_t)constrain(Gout * 255.0, 0.0f, 255.0f);
  slot[CH_B]      = (uint8_t)constrain(Bout * 255.0, 0.0f, 255.0f);
  slot[CH_W]      = (uint8_t)constrain(W * 255.0, 0.0f, 255.0f);
  slot[CH_A]      = (uint8_t)constrain(A * 255.0, 0.0f, 255.0f);
  slot[CH_UV]     = 0;
  slot[CH_STROBE] = 0;   // strobe runs on CH1 so it obeys master and fade
  slot[CH_MODE]   = 0;   // MUST stay 0-9
  slot[CH_SPEED]  = 0;
}

// ---------------------------------------------------------------- effects
// Copied from the candle receiver. Do not retune one without the other.

float fxFlicker(Par& c, bool gusts) {
  unsigned long now = millis();
  if (now - c.lastStep < 16) return constrain(c.level, 0.03f, 1.0f);
  c.lastStep = now;

  int   chance = 2 + (c.speed / 24);
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

float fxEmber(Par& c) {
  unsigned long now = millis();
  if (now - c.lastStep < 16) return constrain(c.level, 0.02f, 0.4f);
  c.lastStep = now;

  if (random(100) < 2) c.target = 0.10 + (random(200) / 1000.0);
  c.level += (c.target - c.level) * 0.03;
  return constrain(c.level, 0.02f, 0.4f);
}

float fxPulse(Par& c) {
  unsigned long period = 4000 - (c.speed * 12);
  float phase = ((millis() + c.phaseOff) % period) / (float)period;
  float b = (sin(phase * TWO_PI - HALF_PI) + 1.0) / 2.0;
  float lo = 1.0 - (c.depth / 255.0);
  return lo + b * (1.0 - lo);
}

float fxStrobe(Par& c) {
  unsigned long period = 500 - (unsigned long)((c.speed / 255.0) * 460);
  if (period < 40) period = 40;
  float duty = 0.08 + (c.depth / 255.0) * 0.52;
  unsigned long onTime = (unsigned long)(period * duty);
  if (onTime < 3) onTime = 3;
  return ((millis() % period) < onTime) ? 1.0 : 0.0;
}

float renderPar(Par& c, uint8_t preset) {
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

static uint8_t maskFor(uint8_t fixture) {
  uint8_t m = 0;
  for (int i = 0; i < NUM_PARS; i++) {
    uint8_t g = PAR_INDEX[i];          // this par's global number, 1-4
    bool hit = false;
    switch (fixture) {
      case 40: hit = true;                        break;   // all pars
      case 45: hit = (g == 1 || g == 2);          break;   // first two
      case 46: hit = (g == 3 || g == 4);          break;   // last two
      case 47: hit = (g == 1 || g == 4);          break;   // inside
      case 48: hit = (g == 2 || g == 3);          break;   // outside
      default:
        if (fixture >= 41 && fixture <= 44) hit = (g == fixture - 40);
        break;
    }
    if (hit) m |= (1 << i);
  }
  return m;
}

static void applyPacket(const CandlePacket& p, uint8_t mask) {
  unsigned long now = millis();
  for (int i = 0; i < NUM_PARS; i++) {
    if (!(mask & (1 << i))) continue;
    Par& c = pr[i];

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

  // Stamp before the fixture filter, so traffic aimed at the candles still
  // proves the link is alive and does not trip our failsafe.
  lastRx = millis();

  // Trim codes set a variable and stop here — they address no par and emit
  // no light, so they are safe to send while a look is running.
  if (p.fixture >= 90 && p.fixture <= 95) {
    float v = p.master / 255.0;
    switch (p.fixture) {
      case 90: whiteW    = v; break;
      case 91: whiteA    = v; break;
      case 92: greenTrim = v; break;
      case 93: trimFlame  = v; break;
      case 94: trimColour = v; break;
      case 95: dimGamma   = (v < 0.05f) ? 0.05f : v; break;
    }
    tuneDirty = true;
    return;
  }

  if (maskFor(p.fixture) == 0) return;

  uint8_t next = (qHead + 1) % PKT_QUEUE;
  if (next == qTail) return;
  memcpy((void*)&pktQueue[qHead], &p, sizeof(p));
  qHead = next;
}

// ---------------------------------------------------------------- setup

void setup() {
  Serial.begin(115200);
  delay(200);

  randomSeed(esp_random());
  for (int i = 0; i < NUM_PARS; i++) pr[i].phaseOff = i * 370;

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
  } else {
    esp_now_register_recv_cb(onRecv);
  }

  dmx_config_t cfg = DMX_CONFIG_DEFAULT;
  dmx_personality_t pers[] = { {CHANS_PER_PAR, "10ch RGBWA UV"} };
  dmx_driver_install(dmxPort, &cfg, pers, 1);
  dmx_set_pin(dmxPort, PIN_TX, PIN_RX, PIN_RTS);

  memset(dmxData, 0, sizeof(dmxData));
  dmxData[0] = 0;                        // DMX start code

  Serial.print("par node "); Serial.print(NODE_ID);
  Serial.print(" driving global pars");
  for (int i = 0; i < NUM_PARS; i++) { Serial.print(" "); Serial.print(PAR_INDEX[i]); }
  Serial.println();
  Serial.println("groups: F40 all, F41-44 single, F45 1+2, F46 3+4, F47 1+4, F48 2+3");
  Serial.print("DMX addresses");
  for (int i = 0; i < NUM_PARS; i++) { Serial.print(" A"); Serial.print(PAR_ADDR[i]); }
  Serial.println();
  Serial.print("my MAC: "); Serial.println(WiFi.macAddress());
  Serial.println("tuning: ww / wa / gt / tf / tc / dg <0-1>, or ?");
  Serial.println("over the air: F90 ww F91 wa F92 gt F93 tf F94 tc F95 dg");
  Serial.printf("ww %.3f  wa %.3f  gt %.3f  tf %.3f  tc %.3f  dg %.3f\n",
                whiteW, whiteA, greenTrim, trimFlame, trimColour, dimGamma);
}

// ---------------------------------------------------------------- tuning
/*
 * Live tuning over this node's USB serial, 115200, newline. Put a candle
 * beside a par, send a steady look from the transmitter, and turn the knobs
 * until they match. Nothing here is saved — write the numbers into the
 * declarations above once you are happy.
 *
 *   ww 0.10     white die share      lower = warmer
 *   wa 1.00     amber share          higher = warmer
 *   gt 0.88     green trim           lower if warm tones go yellow-green
 *   tf 0.30     flame trim on CH1    lower = dimmer candle look
 *   tc 1.00     colour trim on CH1   lower = dimmer saturated looks
 *   dg 1.00     dimmer curve on CH1  lower = lifts the low end
 *   ?           print the current values
 */
static void tuneLine(String ln) {
  ln.trim();
  if (ln.length() == 0) return;

  if (ln == "?") {
    Serial.printf("ww %.3f  wa %.3f  gt %.3f  tf %.3f  tc %.3f  dg %.3f\n",
                  whiteW, whiteA, greenTrim, trimFlame, trimColour, dimGamma);
    return;
  }

  int sp = ln.indexOf(' ');
  if (sp < 0) { Serial.println("? for values, or: ww / wa / gt / tf / tc / dg <0-1>"); return; }

  String key = ln.substring(0, sp);
  float  v   = ln.substring(sp + 1).toFloat();
  v = constrain(v, 0.0f, 2.0f);

  if      (key == "ww") whiteW    = v;
  else if (key == "wa") whiteA    = v;
  else if (key == "gt") greenTrim = v;
  else if (key == "tf") trimFlame  = v;
  else if (key == "tc") trimColour = v;
  else if (key == "dg") dimGamma   = (v < 0.05f) ? 0.05f : v;
  else { Serial.println("unknown. try: ww / wa / gt / tf / tc / dg <0-1>, or ?"); return; }

  Serial.printf("ww %.3f  wa %.3f  gt %.3f  tf %.3f  tc %.3f  dg %.3f\n",
                whiteW, whiteA, greenTrim, trimFlame, trimColour, dimGamma);
}

// ---------------------------------------------------------------- loop

void loop() {
  unsigned long now = millis();

  if (tuneDirty) {
    tuneDirty = false;
    Serial.printf("ww %.3f  wa %.3f  gt %.3f  tf %.3f  tc %.3f  dg %.3f  (air)\n",
                  whiteW, whiteA, greenTrim, trimFlame, trimColour, dimGamma);
  }

  while (Serial.available()) {
    static String buf;
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { tuneLine(buf); buf = ""; }
    else if (buf.length() < 40)    { buf += ch; }
  }

  while (qTail != qHead) {
    CandlePacket p;
    memcpy(&p, (const void*)&pktQueue[qTail], sizeof(p));
    qTail = (qTail + 1) % PKT_QUEUE;
    applyPacket(p, maskFor(p.fixture));
    inFailsafe = false;
  }

  if (lastRx != 0 && now - lastRx > FAILSAFE_MS && !inFailsafe) {
    inFailsafe = true;
    CandlePacket idle = { MAGIC, 0, P_GUST, FAILSAFE_MASTER, FAILSAFE_SPEED,
                          FAILSAFE_DEPTH, FAILSAFE_WARMTH, 100, 0, 0, 0 };
    applyPacket(idle, ALL_PARS);
  }

  for (int i = 0; i < NUM_PARS; i++) {
    Par& c = pr[i];

    float b = renderPar(c, c.preset);

    if (c.fadeLen > 0) {
      unsigned long elapsed = now - c.fadeStart;
      if (elapsed >= c.fadeLen) {
        c.fadeLen = 0;
      } else {
        float blend = (float)elapsed / (float)c.fadeLen;
        // Evaluate the outgoing preset on a scratch copy, or the random walk
        // advances twice per frame and the flicker doubles in speed.
        Par tmp = c;
        float prev = renderPar(tmp, c.prevPreset);
        b = prev * (1.0 - blend) + b * blend;
      }
    }

    float w = (c.warmth / 255.0) * (1.0 - b * 0.35) + (1.0 - b) * 0.2;
    flameDmx(c, b, w, &dmxData[PAR_ADDR[i]]);
  }

  // DMX is a continuous stream, not an event — a frame goes out every pass
  // whether or not anything arrived. This is the one structural difference
  // from the candle receivers.
  dmx_write(dmxPort, dmxData, DMX_SLOTS);
  dmx_send_num(dmxPort, DMX_SLOTS);
  dmx_wait_sent(dmxPort, DMX_TIMEOUT_TICK);

  delay(4);
}
