// PicoEMU web demo: WiFi webserver (STA + fake DHCP, serves :80).
// Attach the gateway and browse to the guest IP to see requests.
#include <WiFi.h>
#include <pico/cyw43_arch.h>

WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("WEB-SRV-START");
  WiFi.begin("Pico-emuNet", "testpassword");
  for (int i = 0; i < 15 && WiFi.status() != WL_CONNECTED; i++) {
    delay(1000);
    cyw43_arch_poll();
  }
  Serial.print("IP=");
  Serial.println(WiFi.localIP());
  server.begin();
  Serial.println("WEB-SRV-LISTEN");
}

void loop() {
  cyw43_arch_poll();
  WiFiClient c = server.available();
  if (c) {
    Serial.println("WEB-SRV-HIT");
    unsigned long t0 = millis();
    while (!c.available() && millis() - t0 < 5000) { cyw43_arch_poll(); delay(10); }
    while (c.available()) { (void)c.read(); }
    c.print("HTTP/1.0 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 11\r\nConnection: close\r\n\r\nhello-pico\n");
    delay(200);
    c.stop();
    Serial.println("WEB-SRV-DONE");
  }
  delay(50);
}
