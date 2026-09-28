/**
 * RV32 WiFi join + DHCP test (RP2350 Hazard3, pico_cyw43_arch_lwip_poll).
 * Joins Pico-emuNet (WPA2, testpassword), waits for a DHCP address.
 * Markers: RV32 JOIN-START / RV32 JOIN err= / RV32 JOIN IP= / RV32 JOIN DONE.
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#define JOIN_SSID "Pico-emuNet"
#define JOIN_PASS "testpassword"

int main() {
    stdio_init_all();

    printf("RV32 JOIN-START\n");
    if (cyw43_arch_init()) {
        printf("RV32 JOIN INIT-FAIL\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();

    printf("RV32 JOINING " JOIN_SSID "\n");
    int err = cyw43_arch_wifi_connect_timeout_ms(JOIN_SSID, JOIN_PASS,
                                                 CYW43_AUTH_WPA2_AES_PSK, 30000);
    printf("RV32 JOIN err=%d status=%d\n", err,
           cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA));
    if (err != 0) {
        printf("RV32 JOIN FAIL\n");
        return 1;
    }
    for (int i = 0; i < 200; i++) {
        cyw43_arch_poll();
        uint32_t ip = ip4_addr_get_u32(netif_ip_addr4(netif_default));
        if (ip != 0) {
            printf("RV32 JOIN IP=%s\n", ip4addr_ntoa(netif_ip_addr4(netif_default)));
            printf("RV32 JOIN DONE\n");
            return 0;
        }
        sleep_ms(100);
    }
    printf("RV32 JOIN NOIP\n");
    return 1;
}
