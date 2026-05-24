/*
 * ============================================================
 *  UWB TAG FIRMWARE  — v4 (smooth edition)
 *  For: Makerfabs ESP32 UWB Pro with Display
 *
 *  Changes from v3:
 *    - Per-anchor EMA (exponential moving average) filter on raw distances
 *    - Outlier rejection: readings more than OUTLIER_FACTOR× the last
 *      smoothed value are discarded (multipath / NLOS spikes)
 *    - Sends smoothed distances to PC instead of raw readings
 * ============================================================
 */

#include <SPI.h>
#include <DW1000Ranging.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ─── CONFIGURE BEFORE FLASHING ────────────────────────────────────────────────
const char* WIFI_SSID = "deep_laptop";
const char* WIFI_PASS = "anshu.com";
const char* PC_IP     = "10.8.60.205";
const int   UDP_PORT  = 5005;
// ──────────────────────────────────────────────────────────────────────────────

// ─── SMOOTHING PARAMETERS ─────────────────────────────────────────────────────
// EMA alpha for distance smoothing (lower = smoother but more lag)
// 0.1 = heavy smooth, 0.3 = moderate. Try 0.15 first.
#define DIST_EMA_ALPHA   0.15f

// Reject a new reading if it differs from the smoothed value by more than
// this factor (e.g. 2.5 = reject if >2.5× or <1/2.5× the last smooth value).
// Catches NLOS / multipath spikes. Set higher (e.g. 4.0) in open environments.
#define OUTLIER_FACTOR   2.5f

// Minimum valid distance (metres). Readings below this are hardware noise.
#define MIN_DIST_M       0.05f

// Maximum valid distance (metres). Readings above this are unreliable.
#define MAX_DIST_M       50.0f
// ──────────────────────────────────────────────────────────────────────────────

// Verified SPI + DW1000 pins for Makerfabs ESP32 UWB Pro with Display
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define PIN_SS    21
#define PIN_RST   27
#define PIN_IRQ   34

// OLED pins
#define OLED_SDA   4
#define OLED_SCL   5
#define OLED_W   128
#define OLED_H    64

char TAG_ADDRESS[] = "7D:00";

#define NUM_ANCHORS 3

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
WiFiUDP udp;

struct AnchorData {
  uint16_t shortAddr;
  float    rawDistance;       // last raw reading (for debug)
  float    smoothedDistance;  // EMA-filtered distance sent to PC
  bool     active;
  bool     initialized;       // true after first valid reading
};

AnchorData anchors[NUM_ANCHORS] = {
  {0x1783, 0.0, 0.0, false, false},  // "83:17" Anchor 1
  {0x1784, 0.0, 0.0, false, false},  // "84:17" Anchor 2
  {0x1785, 0.0, 0.0, false, false},  // "85:17" Anchor 3
};

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 200;

// ─── Helpers ──────────────────────────────────────────────────────────────────

int findAnchorIndex(uint16_t addr) {
  for (int i = 0; i < NUM_ANCHORS; i++) {
    if (anchors[i].shortAddr == addr) return i;
  }
  return -1;
}

int anchorIdForAddr(uint16_t addr) {
  if (addr == 0x1783) return 1;
  if (addr == 0x1784) return 2;
  if (addr == 0x1785) return 3;
  return 0;
}

// Returns true if the reading passes sanity checks and updates the smoothed value.
bool applyFilter(AnchorData &a, float newDist) {
  // Hard range bounds
  if (newDist < MIN_DIST_M || newDist > MAX_DIST_M) return false;

  // Seed the filter on first valid reading
  if (!a.initialized) {
    a.smoothedDistance = newDist;
    a.initialized = true;
    a.rawDistance = newDist;
    return true;
  }

  // Outlier rejection — compare to current smoothed value
  float ratio = newDist / a.smoothedDistance;
  if (ratio > OUTLIER_FACTOR || ratio < (1.0f / OUTLIER_FACTOR)) {
    // Spike detected — discard this reading entirely
    Serial.printf("  [filter] outlier rejected: raw=%.3f smooth=%.3f\n",
                  newDist, a.smoothedDistance);
    return false;
  }

  // EMA update
  a.rawDistance      = newDist;
  a.smoothedDistance = DIST_EMA_ALPHA * newDist
                     + (1.0f - DIST_EMA_ALPHA) * a.smoothedDistance;
  return true;
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("=== UWB Tag ===");
  display.println();
  for (int i = 0; i < NUM_ANCHORS; i++) {
    int aid = anchorIdForAddr(anchors[i].shortAddr);
    if (anchors[i].active) {
      display.printf("A%d: %.2f m\n", aid, anchors[i].smoothedDistance);
    } else {
      display.printf("A%d: ---\n", aid);
    }
  }
  display.println();
  display.printf("%s", WiFi.localIP().toString().c_str());
  display.display();
}

void sendToPC() {
  float d[4] = {0, 0, 0, 0};
  for (int i = 0; i < NUM_ANCHORS; i++) {
    int aid = anchorIdForAddr(anchors[i].shortAddr);
    if (aid >= 1 && aid <= 3 && anchors[i].active && anchors[i].initialized) {
      d[aid] = anchors[i].smoothedDistance;  // ← smoothed, not raw
    }
  }

  char msg[64];
  snprintf(msg, sizeof(msg), "TAG:%.3f,%.3f,%.3f\n", d[1], d[2], d[3]);
  udp.beginPacket(PC_IP, UDP_PORT);
  udp.print(msg);
  udp.endPacket();
  Serial.printf("-> PC: %s", msg);
}

// ─── DW1000Ranging callbacks ──────────────────────────────────────────────────

void newRange() {
  uint16_t addr = DW1000Ranging.getDistantDevice()->getShortAddress();
  float dist    = DW1000Ranging.getDistantDevice()->getRange();

  int idx = findAnchorIndex(addr);
  if (idx >= 0) {
    bool accepted = applyFilter(anchors[idx], dist);
    anchors[idx].active = true;

    int aid = anchorIdForAddr(addr);
    if (accepted) {
      Serial.printf("Anchor %d [%04X]: raw=%.3f smooth=%.3f m\n",
                    aid, addr, dist, anchors[idx].smoothedDistance);
    }
  }
}

void newDevice(DW1000Device* device) {
  uint16_t addr = device->getShortAddress();
  int aid = anchorIdForAddr(addr);
  Serial.printf("New anchor %d [%04X] joined\n", aid, addr);

  int idx = findAnchorIndex(addr);
  if (idx >= 0) {
    anchors[idx].active = true;
    anchors[idx].initialized = false;  // reset filter on reconnect
  }
}

void inactiveDevice(DW1000Device* device) {
  uint16_t addr = device->getShortAddress();
  int aid = anchorIdForAddr(addr);
  Serial.printf("Anchor %d [%04X] inactive\n", aid, addr);

  int idx = findAnchorIndex(addr);
  if (idx >= 0) {
    anchors[idx].active      = false;
    anchors[idx].initialized = false;  // reset filter so stale value isn't reused
  }
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== UWB Tag (Pro with Display) v4-smooth ===");

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed — check SDA/SCL pins");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("UWB Tag Starting...");
  display.display();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi: %s\n", WiFi.localIP().toString().c_str());
  udp.begin(UDP_PORT + 1);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, PIN_SS);

  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  DW1000Ranging.startAsTag(TAG_ADDRESS, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  Serial.println("Tag ready — ranging started (smooth mode)");
  updateDisplay();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
  DW1000Ranging.loop();

  if (millis() - lastSendTime > SEND_INTERVAL_MS) {
    lastSendTime = millis();
    sendToPC();
    updateDisplay();
  }
}
