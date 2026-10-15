// Bench test for the NodeMCU side of the veto-relay panel.
// Hardware UART0 on alternate pins after Serial.swap(): D7 = RX, D8 = TX.
// Answers "PING n" from the Mega with "PONG n", says HELLO every 5 s,
// optionally joins WiFi and reports the IP over the same link.
// Nothing is printed on USB after swap(): watch the Mega's USB monitor.
#include <ESP8266WiFi.h>

const char* WIFI_SSID = "";      // fill in to test WiFi, leave "" to skip
const char* WIFI_PASS = "";

String line;
unsigned long tHello = 0, tWifi = 0;
bool wifiReported = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  delay(50);
  Serial.swap();                  // UART0 -> D7 (RX) / D8 (TX)
  if (strlen(WIFI_SSID)) { WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS); }
  Serial.println("ESP boot");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line.trim();
      if (line.startsWith("PING")) { Serial.print("PONG"); Serial.println(line.substring(4)); }
      else { Serial.print("ECHO "); Serial.println(line); }
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      line = "";
    } else if (c != '\r' && line.length() < 80) line += c;
  }
  if (millis() - tHello > 5000) {
    tHello = millis();
    Serial.print("HELLO up="); Serial.print(millis() / 1000); Serial.print("s rssi=");
    Serial.println(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
  }
  if (strlen(WIFI_SSID) && !wifiReported && WiFi.status() == WL_CONNECTED) {
    wifiReported = true;
    Serial.print("WIFI "); Serial.println(WiFi.localIP());
  }
}
