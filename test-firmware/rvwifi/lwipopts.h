#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Self-contained lwIP options for rvwifi_join (pico_cyw43_arch_lwip_poll,
// RP2350 RISC-V). No arduino/pico-examples dependencies.

#define NO_SYS                        1
#define LWIP_SOCKET                   0
#define MEM_LIBC_MALLOC               0

#define MEM_ALIGNMENT                 4
#define MEM_SIZE                      16384
#define MEMP_NUM_TCP_SEG              32
#define MEMP_NUM_ARP_QUEUE            10
#define PBUF_POOL_SIZE                24
#define LWIP_ARP                      1
#define LWIP_ETHERNET                 1
#define LWIP_ICMP                     1
#define LWIP_RAW                      1
#define TCP_WND                       (8 * TCP_MSS)
#define TCP_MSS                       1460
#define TCP_SND_BUF                   (8 * TCP_MSS)
#define LWIP_NETIF_STATUS_CALLBACK    1
#define LWIP_NETIF_LINK_CALLBACK      1
#define LWIP_NETIF_HOSTNAME           1
#define LWIP_NETCONN                  0
#define LWIP_DHCP                     1
#define LWIP_DHCP_CHECK_LINK_UP       1
#define LWIP_IPV4                     1
#define LWIP_TCP                      1
#define LWIP_UDP                      1
#define LWIP_DNS                      1
#define LWIP_TCP_KEEPALIVE            1

#endif
