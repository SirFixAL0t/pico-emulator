#include <W6300lwIP.h>
// pico-w6300 DHCP prove-out (WIZnet W6300-EVB-Pico/Pico2, QSPI-single on
// SPI0, CS16/RST22/INT15). Mirrors ethdhcp.ino (W5500, CS17/RST20/INT21).
// FQBN pico (M0+): rp2040:rp2040:wiznet_w6300_evb_pico (if available)
// FQBN pico2 (M33): rp2040:rp2040:wiznet_w6300_evb_pico2 (if available)
// Prints ETH-BEGIN-OK, then ETH-IP=<addr> once DHCP binds (or (IP unset)).
// Emulator run (needs a live peer — Arduino DHCP has no dead-peer marker):
//   ./build/picoemu /tmp/ethdhcp6300/out/ethdhcp6300.ino.uf2 -board pico-w6300 \
//       -net-peer /tmp/ethdhcp6300.sock
//   python3 test-firmware/dhcp_peer_test.py /tmp/ethdhcp6300.sock
// Expect: ALL DHCP CHECKS PASSED + ETH-IP=192.168.4.2 on UART.
// NOTE: requires the Wiznet W6300 Arduino library (W6300lwIP); if the
// core/lib is not installed the sketch does not compile — the in-tree
// eth_dhcp6300 guests remain the offline-capable prove-out.
Wiznet6300lwIP eth(16, SPI, 15);

static unsigned long spin_until(unsigned long ms) {
  // Busy-pump instead of WFI sleep: keeps guest time at wall pace so the
  // async-context lwIP pump + DHCP timers track the wall-ms peer replies.
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { /* spin */ }
  return millis();
}

void setup() {
  Serial.begin(115200);
  spin_until(2000);
  Serial.println("ETHDHCP-START");
  bool ok = eth.begin();  // DHCP (default)
  Serial.println(ok ? "ETH-BEGIN-OK" : "ETH-BEGIN-FAIL");
}

void loop() {
  static unsigned long last = 0;
  static int phase = 0;
  unsigned long now = millis();
  if (now - last > 2000) {
    last = now;
    phase++;
    Serial.print("ETH-TICK conn=");
    Serial.print(eth.connected() ? 1 : 0);
    Serial.print(" ip=");
    Serial.println(eth.localIP());
    spin_until(1500);
    if (phase > 30) { /* stop after ~1 min */ while (1) { delay(5000); } }
  }
}
