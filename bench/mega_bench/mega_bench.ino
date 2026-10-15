// Bench test for the veto-relay panel: Mega 2560 + MCUFRIEND 3.5" TFT +
// HL-56S relays + NodeMCU on Serial1. No sensors yet.
// Libraries: MCUFRIEND_kbv, Adafruit_GFX, TouchScreen (same as diesel project).
//
// What it does:
//  - relays IN1..IN6 on D22,24,26,28,30,32 (active LOW), all released at boot
//  - relays 2..6 step one at a time every 2 s (relay 1 = DZT loop is NOT cycled)
//  - touch anywhere on the screen toggles relay 1 (DZT loop)
//  - sends "PING n" to the ESP on Serial1 every second, shows the reply
//  - USB Serial 115200 mirrors everything
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>

MCUFRIEND_kbv tft;
const int XP = 9, XM = A3, YP = A2, YM = 8;
const int TS_LEFT = 927, TS_RT = 63, TS_TOP = 908, TS_BOT = 125;
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

const uint8_t RELAY[6] = {22, 24, 26, 28, 30, 32};
bool relay1 = false;            // DZT loop channel
uint8_t cycleIdx = 1;           // 1..5 -> relays 2..6
unsigned long tCycle = 0, tPing = 0, tTouch = 0;
unsigned long pingN = 0, pongN = 0, lastRxMs = 0;
String rxLine;

void setRelay(uint8_t i, bool on) { digitalWrite(RELAY[i], on ? LOW : HIGH); }

void drawStatus() {
  tft.fillRect(0, 60, 480, 260, TFT_BLACK);
  tft.setCursor(10, 70);  tft.setTextSize(3);
  tft.print("DZT relay 1: "); tft.print(relay1 ? "ON (loop open)" : "off");
  tft.setCursor(10, 110); tft.print("cycling relay: "); tft.print(cycleIdx + 1);
  bool link = (millis() - lastRxMs) < 3000 && lastRxMs > 0;
  tft.setCursor(10, 150); tft.setTextColor(link ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.print(link ? "ESP link OK" : "ESP no link");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 190); tft.print("ping "); tft.print(pingN); tft.print(" pong "); tft.print(pongN);
  tft.setCursor(10, 240); tft.setTextSize(2); tft.print("touch = toggle DZT relay");
}

void setup() {
  for (uint8_t i = 0; i < 6; i++) { digitalWrite(RELAY[i], HIGH); pinMode(RELAY[i], OUTPUT); }
  Serial.begin(115200);
  Serial1.begin(9600);
  uint16_t id = tft.readID();
  if (id == 0xD3D3) id = 0x9486;      // common fallback for 3.5" clones
  tft.begin(id);
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(3); tft.setCursor(10, 10); tft.print("veto-relay bench");
  tft.setTextSize(2); tft.setCursor(300, 20); tft.print("TFT 0x"); tft.print(id, HEX);
  Serial.print(F("bench start, TFT id 0x")); Serial.println(id, HEX);
  drawStatus();
}

void loop() {
  unsigned long now = millis();

  // relay cycle 2..6
  if (now - tCycle > 2000) {
    tCycle = now;
    for (uint8_t i = 1; i < 6; i++) setRelay(i, false);
    cycleIdx = (cycleIdx % 5) + 1;      // 1..5
    setRelay(cycleIdx, true);
    Serial.print(F("relay ")); Serial.print(cycleIdx + 1); Serial.println(F(" on"));
    drawStatus();
  }

  // ping ESP
  if (now - tPing > 1000) {
    tPing = now; pingN++;
    Serial1.print(F("PING ")); Serial1.println(pingN);
  }

  // read ESP
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      rxLine.trim();
      Serial.print(F("ESP: ")); Serial.println(rxLine);
      if (rxLine.startsWith("PONG")) { pongN = rxLine.substring(5).toInt(); lastRxMs = now; }
      else lastRxMs = now;
      rxLine = "";
    } else if (c != '\r' && rxLine.length() < 80) rxLine += c;
  }

  // touch -> toggle relay 1
  TSPoint p = ts.getPoint();
  pinMode(XM, OUTPUT); pinMode(YP, OUTPUT);   // restore display lines
  if (p.z > 100 && p.z < 1000 && now - tTouch > 500) {
    tTouch = now;
    relay1 = !relay1; setRelay(0, relay1);
    Serial.print(F("DZT relay 1 ")); Serial.println(relay1 ? F("ON") : F("off"));
    drawStatus();
  }
}
