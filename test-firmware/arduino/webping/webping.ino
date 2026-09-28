// PicoEMU web demo: ping own gateway IP (offline-safe, no gateway needed).
// With -nodhcp the fake server owns 192.168.4.1 and answers pings.
#include <WiFi.h>
#include <pico/cyw43_arch.h>

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("WIFI-PING-START");
  WiFi.begin("Pico-emuNet", "testpassword");
  for (int i = 0; i < 15 && WiFi.status() != WL_CONNECTED; i++) {
    delay(1000);
    cyw43_arch_poll();
  }
  Serial.print("IP=");
  Serial.println(WiFi.localIP());
  int t = WiFi.ping(WiFi.gatewayIP());
  Serial.print("PING-GW=");
  Serial.println(t);
  Serial.println("WIFI-PING-DONE");
}

void loop() { delay(10000); }
