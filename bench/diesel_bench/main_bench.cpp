// Bench build for the veto-relay panel, living inside diesel_heater_interface.
// Build with the [env:bench] environment (platformio.ini snippet in README):
// -DBENCH compiles this file and skips main.cpp. Display.cpp and the CAN
// sources still compile (CAN hardware absent, never touched here).
//
//  - relays IN1..IN6 on D22,24,26,28,30,32 (active LOW), all released at boot
//  - six on-screen buttons toggle the relays; R1 = DZT loop
//  - CYCLE button: relays 2..6 step every 2 s (R1 never cycled)
//  - PING n to the ESP on Serial1 every second, link state and reply shown
//  - USB Serial 115200 mirrors everything
#ifdef BENCH
#include <Arduino.h>
#include <TouchScreen.h>
#include <MCUFRIEND_kbv.h>
#include "Display.h"

MCUFRIEND_kbv tft;
const int XP = 9, XM = A3, YP = A2, YM = 8;
const int TS_LEFT = 927, TS_RT = 63, TS_TOP = 908, TS_BOT = 125;
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Display display(tft, ts);   // used for initilize() only

const uint8_t RELAY[6] = {22, 24, 26, 28, 30, 32};
bool relayOn[6] = {false, false, false, false, false, false};
bool cycling = false;
uint8_t cycleIdx = 1;
unsigned long tCycle = 0, tPing = 0;
unsigned long pingN = 0, pongN = 0, lastRxMs = 0;
String rxLine, lastMsg;

Adafruit_GFX_Button btn[6], btnCycle;
char lbl[6][8] = {"R1 DZT", "R2", "R3", "R4", "R5", "R6"};
char lblCycle[6] = "CYCLE";
int pixel_x, pixel_y;

bool touchGetXY() {
  TSPoint p = ts.getPoint();
  pinMode(YP, OUTPUT); pinMode(XM, OUTPUT);
  digitalWrite(YP, HIGH); digitalWrite(XM, HIGH);
  bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
  if (pressed) {
    pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width());
    pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
  }
  return pressed;
}

void setRelay(uint8_t i, bool on) {
  relayOn[i] = on;
  digitalWrite(RELAY[i], on ? LOW : HIGH);
  Serial.print(F("relay ")); Serial.print(i + 1); Serial.println(on ? F(" ON") : F(" off"));
}

void drawRelayButton(uint8_t i) {
  btn[i].initButton(&tft, 60 + i * 75, 60, 70, 50, WHITE, relayOn[i] ? GREEN : GRAY, BLACK, lbl[i], 2);
  btn[i].drawButton(false);
}

void drawStatus() {
  tft.fillRect(0, 100, 480, 110, BLACK);
  tft.setTextSize(2); tft.setTextColor(WHITE, BLACK);
  bool link = lastRxMs > 0 && (millis() - lastRxMs) < 3000;
  tft.setCursor(10, 110); tft.setTextColor(link ? GREEN : RED, BLACK);
  tft.print(link ? F("ESP link OK   ") : F("ESP no link   "));
  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(10, 135); tft.print(F("ping ")); tft.print(pingN); tft.print(F("  pong ")); tft.print(pongN);
  tft.setCursor(10, 160); tft.print(F("last: ")); tft.print(lastMsg.substring(0, 30));
  tft.setCursor(10, 185); tft.print(F("cycle ")); tft.print(cycling ? F("ON ") : F("off"));
  if (cycling) { tft.print(F(" -> R")); tft.print(cycleIdx + 1); }
}

void setup() {
  for (uint8_t i = 0; i < 6; i++) { digitalWrite(RELAY[i], HIGH); pinMode(RELAY[i], OUTPUT); }
  Serial.begin(115200);
  Serial1.begin(9600);
  display.initilize();
  tft.setTextSize(2); tft.setTextColor(WHITE, BLACK);
  tft.setCursor(10, 10); tft.print(F("veto-relay bench"));
  for (uint8_t i = 0; i < 6; i++) drawRelayButton(i);
  btnCycle.initButton(&tft, 400, 250, 140, 50, WHITE, CYAN, BLACK, lblCycle, 2);
  btnCycle.drawButton(false);
  drawStatus();
  Serial.println(F("bench start"));
}

void loop() {
  unsigned long now = millis();
  bool down = touchGetXY();

  for (uint8_t i = 0; i < 6; i++) {
    btn[i].press(down && btn[i].contains(pixel_x, pixel_y));
    if (btn[i].justPressed()) { setRelay(i, !relayOn[i]); drawRelayButton(i); }
  }
  btnCycle.press(down && btnCycle.contains(pixel_x, pixel_y));
  if (btnCycle.justPressed()) {
    cycling = !cycling;
    if (!cycling) for (uint8_t i = 1; i < 6; i++) { setRelay(i, false); drawRelayButton(i); }
    drawStatus();
  }

  if (cycling && now - tCycle > 2000) {
    tCycle = now;
    setRelay(cycleIdx, false); drawRelayButton(cycleIdx);
    cycleIdx = (cycleIdx % 5) + 1;          // 1..5 -> R2..R6
    setRelay(cycleIdx, true); drawRelayButton(cycleIdx);
    drawStatus();
  }

  if (now - tPing > 1000) {
    tPing = now; pingN++;
    Serial1.print(F("PING ")); Serial1.println(pingN);
    drawStatus();
  }

  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      rxLine.trim();
      Serial.print(F("ESP: ")); Serial.println(rxLine);
      lastMsg = rxLine; lastRxMs = now;
      if (rxLine.startsWith("PONG")) pongN = rxLine.substring(5).toInt();
      rxLine = "";
    } else if (c != '\r' && rxLine.length() < 80) rxLine += c;
  }
}
#endif
