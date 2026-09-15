/*
 * candle_transmitter.ino
 *
 * Stays plugged into the laptop over USB. Reads a one-line text command,
 * broadcasts it to every candelabra over ESP-NOW.
 *
 * Protocol — one line, terminated by newline:
 *
 *   F<fixture> P<preset> M<master> S<speed> D<depth> W<warmth> T<fade>
 *
 *   F  0 = all, 1 = candelabra A, 2 = candelabra B
 *   P  0 blackout, 1 steady, 2 flicker, 3 gust, 4 ember, 5 pulse, 6 strobe
 *   M  master intensity 0-255
 *   S  speed 0-255
 *   D  depth 0-255
 *   W  warmth 0-255
 *   T  crossfade time, hundredths of a second (0 = snap, 100 = 1s)
 *   H  hue 0-255 around the colour wheel
 *   C  colour saturation: 0 = candle flame, 255 = fully saturated hue
 *
 * Any field may be omitted; the last value is kept.
 *
 * Examples you can paste straight into the Serial Monitor:
 *   F0 P2 M90 S120 D140 W170 T50     -> everything to a warm flicker, 0.5s fade
 *   F0 P6 M255 S200 T0               -> hard strobe, snap
 *   F1 P4 M40 T200                   -> candelabra A embers down over 2s
 *   F0 P0 T150                       -> blackout over 1.5s
 *
 * Board: ESP32 Dev Module.  Requires esp32 core 3.x.
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1     // must match the receivers
#define HEARTBEAT_MS 500   // resend current state this often, so receivers
                           // hold their look and any that reboot catch up

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t fixture;
  uint8_t preset;
  uint8_t master;
  uint8_t speed;
  uint8_t depth;
  uint8_t warmth;
  uint8_t fade;
  uint8_t hue;
  uint8_t sat;
  uint8_t boost;    // 0 = flame as always, 255 = all four dies at full
} CandlePacket;

#define MAGIC 0xC1

// Broadcast — no pairing, no MAC addresses to chase at a venue.
uint8_t broadcastAddr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Current values, so omitted fields persist
CandlePacket pkt = { MAGIC, 0, 2, 255, 170, 220, 0, 0, 0, 0, 0 };

String line;
unsigned long lastSend = 0;

// ---------------------------------------------------------------- setup

void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddr, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("add_peer FAILED");
    return;
  }

  Serial.println();
  Serial.println("transmitter ready");
  Serial.println("try: F0 P3 M120 S140 D180 W180 T80");
}

// ---------------------------------------------------------------- parse

// Pull the number following a letter, e.g. 'M' in "F0 M200 P3"
bool field(const String& s, char key, uint8_t& out) {
  int i = s.indexOf(key);
  if (i < 0) return false;
  int v = s.substring(i + 1).toInt();
  out = (uint8_t)constrain(v, 0, 255);
  return true;
}

void send(bool quiet = false) {
  esp_err_t r = esp_now_send(broadcastAddr, (uint8_t*)&pkt, sizeof(pkt));
  lastSend = millis();

  if (quiet && r == ESP_OK) return;   // don't spam the monitor with heartbeats

  Serial.print(r == ESP_OK ? "sent  " : "FAIL  ");
  Serial.print("F"); Serial.print(pkt.fixture);
  Serial.print(" P"); Serial.print(pkt.preset);
  Serial.print(" M"); Serial.print(pkt.master);
  Serial.print(" S"); Serial.print(pkt.speed);
  Serial.print(" D"); Serial.print(pkt.depth);
  Serial.print(" W"); Serial.print(pkt.warmth);
  Serial.print(" T"); Serial.print(pkt.fade);
  Serial.print(" H"); Serial.print(pkt.hue);
  Serial.print(" C"); Serial.print(pkt.sat);
  Serial.print(" B"); Serial.println(pkt.boost);
}

// ---------------------------------------------------------------- loop

void loop() {

  // Heartbeat. Receivers drop to their idle look if they hear nothing for
  // a few seconds, so keep restating the current state.
  if (millis() - lastSend > HEARTBEAT_MS) send(true);

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      line.trim();
      line.toUpperCase();
      if (line.length() > 0) {
        field(line, 'F', pkt.fixture);
        field(line, 'P', pkt.preset);
        field(line, 'M', pkt.master);
        field(line, 'S', pkt.speed);
        field(line, 'D', pkt.depth);
        field(line, 'W', pkt.warmth);
        field(line, 'T', pkt.fade);
        field(line, 'H', pkt.hue);
        field(line, 'C', pkt.sat);
        field(line, 'B', pkt.boost);
        send();
      }
      line = "";
    } else {
      line += c;
      if (line.length() > 80) line = "";   // runaway guard
    }
  }
}
