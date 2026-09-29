/*
 * W6300 Ethernet Controller (QSPI Device Plugin)
 *
 * Register-level model of the WIZnet W6300 hardwired dual IPv4/IPv6
 * TCP/IP controller, in the same emulator role as the W5500 model
 * (src/w5500.c): socket commands update status registers, MACRAW
 * socket 0 joins the shared vnet bus (single gateway with CYW43 WiFi),
 * live mode dials real host TCP/UDP sockets, and the WASM proxy pump
 * queues CONNECT/LISTEN/CLOSE/SEND for JS.
 *
 * Guest-visible hardware (W6300 datasheet DS v1.0.2 + WIZnet
 * ioLibrary_Driver Ethernet/W6300/w6300.h):
 * - QSPI SINGLE mode frame per CS: opcode(1B)=[block:5][R/W:1][mode:2]
 *   + addr(2B BE) + dummy(1B) + N data bytes. Block encoding matches
 *   the W5500 BSB (0=common, 1+4N=sock regs, 2+4N=TX, 3+4N=RX).
 * - 16-bit register addresses; CIDR0=0x61/CIDR1=0x00/CIDR2=0x11;
 *   SYSR lock bits (CHPL7/NETL6/PHYL5, all locked at reset) with
 *   unlock magic CHPLCKR=0xCE NETLCKR=0x3A PHYLCKR=0x53;
 *   PHYSR with opposite speed/duplex polarity vs W5500;
 *   Sn_CR=0x0010 Sn_IR=0x0020(+IRCLR 0x0028 W1C) Sn_SR=0x0030;
 *   4KB TX/RX per socket; MACRAW=0x07; CLOSE=0x10 (≠ W5500 0x08).
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "w6300.h"
#include "vnet.h"

/* Bridge trace (file scope: RECV path at ~line 460 and the bridge
 * section both log). Compile with -DPICOEMU_W6300_TRACE=1 for
 * unconditional logging, or set PICOEMU_W6300_TRACE=1 at runtime. */
#ifdef PICOEMU_W6300_TRACE
#define W6300_TR(...) do { fprintf(stderr, "[W6300-QSPI] " __VA_ARGS__); fputc('\n', stderr); } while (0)
#else
static int w6300_tr_enabled(void) {
    static int cached = -1;
    if (cached < 0) cached = getenv("PICOEMU_W6300_TRACE") ? 1 : 0;
    return cached;
}
#define W6300_TR(...) do { if (w6300_tr_enabled()) { fprintf(stderr, "[W6300-QSPI] " __VA_ARGS__); fputc('\n', stderr); } } while (0)
#endif

/* WASM proxy queue (same framing as the W5500 pump; separate queue so
 * the two chips never interleave). Drained by JS through
 * picoemu_w6300_pop_tx(). */
#define W6300_WS_TX_SIZE 8192
static uint8_t w6300_ws_tx_buf[W6300_WS_TX_SIZE];
static int w6300_ws_tx_head = 0, w6300_ws_tx_tail = 0;

void w6300_ws_tx_push(const uint8_t *data, int len) {
    if (!data || len <= 0) return;
    for (int i = 0; i < len; i++) {
        int nxt = (w6300_ws_tx_head + 1) % W6300_WS_TX_SIZE;
        if (nxt == w6300_ws_tx_tail) break;
        w6300_ws_tx_buf[w6300_ws_tx_head] = data[i];
        w6300_ws_tx_head = nxt;
    }
}

int picoemu_w6300_pop_tx(uint8_t *out, int maxlen) {
    int n = 0;
    while (n < maxlen && w6300_ws_tx_tail != w6300_ws_tx_head) {
        out[n++] = w6300_ws_tx_buf[w6300_ws_tx_tail];
        w6300_ws_tx_tail = (w6300_ws_tx_tail + 1) % W6300_WS_TX_SIZE;
    }
    return n;
}

int picoemu_w6300_tx_len(void) {
    int n = w6300_ws_tx_head - w6300_ws_tx_tail;
    if (n < 0) n += W6300_WS_TX_SIZE;
    return n;
}

/* ========================================================================
 * Block decoding (same 5-bit encoding as W5500 BSB)
 * ======================================================================== */

static int blk_type(uint8_t bsb) {
    if (bsb == 0) return 0;
    return ((bsb - 1) & 3) + 1;  /* 1=reg, 2=TX, 3=RX */
}

static int blk_socket(uint8_t bsb) {
    if (bsb == 0) return -1;
    return (bsb - 1) >> 2;
}

/* ========================================================================
 * Socket command processing
 * ======================================================================== */

static void w6300_board_refresh_int(void);

static void set_sock_nonblock(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1)
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

/* Socket family actually dialled on the host fd (set at OPEN, may be
 * upgraded AF_INET->AF_INET6 by a dual-stack CONNECT/CONNECT6). Guests
 * see only Sn_SR; this keeps native + WASM pump + poll agreeing. */
static int w6300_sock_host_v6(w6300_socket_t *s) {
    int fam = 0;
    socklen_t sl = sizeof(fam);
    if (s->host_fd >= 0 &&
        getsockopt(s->host_fd, SOL_SOCKET, SO_DOMAIN, &fam, &sl) == 0)
        return fam == AF_INET6;
    if (s->host_listen_fd >= 0 &&
        getsockopt(s->host_listen_fd, SOL_SOCKET, SO_DOMAIN, &fam, &sl) == 0)
        return fam == AF_INET6;
    return 0;
}

/* Re-create the host socket in a new family, preserving nonblocking
 * mode. Used when a dual-stack (TCPD/UDPD, opened AF_INET by default)
 * socket upgrades to IPv6 on CONNECT/CONNECT6 with a v6 destination.
 * Returns 0 on success, -1 on failure (old fd already closed). */
static int w6300_sock_recreate(int *fdp, int family, int type, int proto) {
    int old = *fdp;
    if (old >= 0) close(old);
    *fdp = socket(family, type, proto);
    if (*fdp < 0) return -1;
    set_sock_nonblock(*fdp);
    return 0;
}

/* Cross-check an IPv6 host fd we believe we own: getsockname must
 * report AF_INET6 (guards fd-reuse / wrong-family regressions). */
static int w6300_host_fd_is_v6(int fd) {
    if (fd < 0) return 0;
    struct sockaddr_storage ss;
    socklen_t sl = sizeof(ss);
    if (getsockname(fd, (struct sockaddr *)&ss, &sl) != 0) return 0;
    return ss.ss_family == AF_INET6;
}

static void w6300_sock_apply_ttl_tos(w6300_socket_t *s) {
    /* Sn_TTLR/Sn_TOSR apply at dial time (live). TTL 0 = default 64. */
    if (s->host_fd < 0) return;
    uint8_t ttl = s->regs[W6300_Sn_TTLR];
    if (ttl == 0) ttl = 64;
    int fam = 0;
    socklen_t fl = sizeof(fam);
    if (getsockopt(s->host_fd, SOL_SOCKET, SO_DOMAIN, &fam, &fl) != 0)
        fam = AF_INET;
    if (fam == AF_INET6) {
        setsockopt(s->host_fd, IPPROTO_IPV6, IPV6_UNICAST_HOPS,
                   &ttl, sizeof(ttl));
        int tclass = s->regs[W6300_Sn_TOSR];
        setsockopt(s->host_fd, IPPROTO_IPV6, IPV6_TCLASS,
                   &tclass, sizeof(tclass));
    } else {
        setsockopt(s->host_fd, IPPROTO_IP, IP_TTL, &ttl, sizeof(ttl));
        int tos = s->regs[W6300_Sn_TOSR];
        setsockopt(s->host_fd, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));
    }
}

/* Retry budget: Sn_RCR counts attempts, RTR is the per-attempt timeout
 * (units of 100us). poll() is nonblocking, so the engine tracks poll
 * ticks per socket; a SYNSENT socket whose ticks exceed RCR attempts
 * (scaled: 1 tick ~= 1ms, attempt = RTR*100us) raises TIMEOUT + CLOSED
 * exactly like silicon exhausting its ARP/TCP retransmits. RCR=0 means
 * retry forever (ioLibrary: 0 = no limit); RTR=0 means the default 2s.
 * Progress (writable/connected) clears the counter. */
#define W6300_POLL_TICK_MS 1
static void w6300_sock_retry_tick(w6300_t *dev, int sock);

static void w6300_build_addr(w6300_socket_t *s, struct sockaddr_in *addr) {
    memset(addr, 0, sizeof(*addr));
    addr->sin_family = AF_INET;
    uint32_t ip = ((uint32_t)s->regs[W6300_Sn_DIPR0] << 24) |
                  ((uint32_t)s->regs[W6300_Sn_DIPR0 + 1] << 16) |
                  ((uint32_t)s->regs[W6300_Sn_DIPR0 + 2] << 8) |
                  (uint32_t)s->regs[W6300_Sn_DIPR0 + 3];
    addr->sin_addr.s_addr = htonl(ip);
    uint16_t port = ((uint16_t)s->regs[W6300_Sn_DPORTR0] << 8) |
                    s->regs[W6300_Sn_DPORTR0 + 1];
    addr->sin_port = htons(port);
}

static void w6300_close_host_sock(w6300_socket_t *s) {
    if (s->host_fd >= 0) {
        close(s->host_fd);
        s->host_fd = -1;
    }
    if (s->host_listen_fd >= 0) {
        close(s->host_listen_fd);
        s->host_listen_fd = -1;
    }
}

/* ========================================================================
 * MACRAW gateway path (single-gateway Ethernet, shared vnet bus)
 * ======================================================================== */

static void w6300_macraw_vnet_rx(void *ctx, const uint8_t *frame, int len) {
    w6300_t *dev = NULL;
    int sock = -1;
    extern void w6300_macraw_dispatch(void *ctx, w6300_t **dev_out, int *sock_out);
    w6300_macraw_dispatch(ctx, &dev, &sock);
    if (!dev || sock < 0 || sock >= W6300_NUM_SOCKETS) return;
    if (!frame || len < 14 || len > 1514) return;
    w6300_socket_t *s = &dev->sockets[sock];
    if (s->regs[W6300_Sn_MR] != W6300_MR_MACRAW ||
        s->regs[W6300_Sn_SR] != W6300_SOCK_MACRAW)
        return;
    uint16_t rx_rsr = ((uint16_t)s->regs[W6300_Sn_RX_RSR0] << 8) |
                      s->regs[W6300_Sn_RX_RSR0 + 1];
    uint16_t free_space = W6300_RX_BUF_SIZE - rx_rsr;
    uint16_t need = (uint16_t)(len + 2);
    if (free_space < need) return;
    W6300_TR("VNET-RX len=%d rsr=%u base=0x%04X", len, rx_rsr, s->rx_base);
    uint16_t rx_wr = (uint16_t)(s->rx_base + rx_rsr);
    uint16_t stored = (uint16_t)(len + 2);  /* prefix INCLUDES its 2 bytes */
    s->rx_buf[rx_wr % W6300_RX_BUF_SIZE] = (uint8_t)((stored >> 8) & 0xFF);
    s->rx_buf[(rx_wr + 1) % W6300_RX_BUF_SIZE] = (uint8_t)(stored & 0xFF);
    for (int i = 0; i < len; i++)
        s->rx_buf[(rx_wr + 2 + (uint16_t)i) % W6300_RX_BUF_SIZE] = frame[i];
    rx_wr = (uint16_t)(rx_wr + need);
    s->regs[W6300_Sn_RX_WR0]     = (rx_wr >> 8) & 0xFF;
    s->regs[W6300_Sn_RX_WR0 + 1] = rx_wr & 0xFF;
    rx_rsr = (uint16_t)(rx_rsr + need);
    s->regs[W6300_Sn_RX_RSR0]     = (rx_rsr >> 8) & 0xFF;
    s->regs[W6300_Sn_RX_RSR0 + 1] = rx_rsr & 0xFF;
    s->regs[W6300_Sn_IR] |= W6300_IR_RECV;
    w6300_board_refresh_int();
}

#define W6300_MACRAW_MAXDEVS 2
static struct {
    w6300_t *dev;
    int sock;
    uint8_t mac[6];
} w6300_macraw_devs[W6300_MACRAW_MAXDEVS];
static int w6300_macraw_ndevs = 0;

static int w6300_gw_enable = 1;
void w6300_gw_enable_set(int on) { w6300_gw_enable = on ? 1 : 0; }
int w6300_gw_enabled(void) { return w6300_gw_enable; }

void w6300_macraw_dispatch(void *ctx, w6300_t **dev_out, int *sock_out) {
    intptr_t idx = (intptr_t)ctx;
    if (idx < 0 || idx >= w6300_macraw_ndevs) {
        *dev_out = NULL; *sock_out = -1;
        return;
    }
    *dev_out = w6300_macraw_devs[idx].dev;
    *sock_out = w6300_macraw_devs[idx].sock;
}

int w6300_macraw_attach(w6300_t *dev, int sock) {
    if (!dev || sock < 0 || sock >= W6300_NUM_SOCKETS) return -1;
    if (!w6300_gw_enable) return -1;
    uint8_t mac[6];
    for (int i = 0; i < 6; i++) mac[i] = dev->common[W6300_SHAR0 + i];
    for (int i = 0; i < w6300_macraw_ndevs; i++) {
        if (w6300_macraw_devs[i].dev == dev && w6300_macraw_devs[i].sock == sock) {
            if (memcmp(w6300_macraw_devs[i].mac, mac, 6) != 0) {
                memcpy(w6300_macraw_devs[i].mac, mac, 6);
                if (dev->vnet_port >= 0)
                    vnet_update_port_mac(dev->vnet_port, mac);
            }
            return dev->vnet_port;
        }
    }
    if (w6300_macraw_ndevs >= W6300_MACRAW_MAXDEVS) return -1;
    if (!vnet.enabled) vnet_init();
    int idx = w6300_macraw_ndevs;
    w6300_macraw_devs[idx].dev = dev;
    w6300_macraw_devs[idx].sock = sock;
    memcpy(w6300_macraw_devs[idx].mac, mac, 6);
    w6300_macraw_ndevs++;
    int port = vnet_register_port("w6300-macraw", VNET_PORT_W6300, mac,
                                  w6300_macraw_vnet_rx, (void *)(intptr_t)idx);
    dev->vnet_port = port;
    /* WASM note: picoemu_wasm.c provides w6300_macraw_vnet_mark() to set
     * wasm_vnet_on when the MACRAW path brings vnet up itself. Weak ref
     * keeps native/test/WASM all linking (native has no such symbol). */
    extern void w6300_macraw_vnet_mark(void) __attribute__((weak));
    if (w6300_macraw_vnet_mark) w6300_macraw_vnet_mark();
    fprintf(stderr, "[W6300] MACRAW socket %d on vnet port %d\n", sock, port);
    return port;
}

static void w6300_macraw_send(w6300_t *dev, int sock) {
    w6300_socket_t *s = &dev->sockets[sock];
    uint16_t tx_wr = ((uint16_t)s->regs[W6300_Sn_TX_WR0] << 8) |
                     s->regs[W6300_Sn_TX_WR0 + 1];
    uint16_t tx_rd = ((uint16_t)s->regs[W6300_Sn_TX_RD0] << 8) |
                     s->regs[W6300_Sn_TX_RD0 + 1];
    /* Canonical length: TX_WR - TX_RD (ring sequence numbers, NOT the
     * dirty tracker). The dirty tracker is a byte-accumulation hint
     * for non-wrapping writes; the guest's TX_WR register is the real
     * write pointer. Using dirty_len here misfires when the DHCP
     * payload wraps past the tracker's linear run (342B DISCOVER at
     * a nonzero TX_WR reads short and the frame never hits vnet). */
    uint16_t data_len = (uint16_t)(tx_wr - tx_rd);
    uint16_t base = tx_rd;
    if (data_len > W6300_TX_BUF_SIZE) data_len = W6300_TX_BUF_SIZE;
    if (data_len < 14 || data_len > 1514) {
        s->regs[W6300_Sn_TX_RD0] = s->regs[W6300_Sn_TX_WR0];
        s->regs[W6300_Sn_TX_RD0 + 1] = s->regs[W6300_Sn_TX_WR0 + 1];
        s->regs[W6300_Sn_TX_FSR0] = (W6300_TX_BUF_SIZE >> 8) & 0xFF;
        s->regs[W6300_Sn_TX_FSR0 + 1] = W6300_TX_BUF_SIZE & 0xFF;
        s->regs[W6300_Sn_IR] |= W6300_IR_SENDOK;
        s->tx_dirty_valid = 0;
        s->tx_dirty_len = 0;
        return;
    }
    uint8_t frame[1514];
    for (uint16_t i = 0; i < data_len; i++)
        frame[i] = s->tx_buf[(uint16_t)(base + i) % W6300_TX_BUF_SIZE];
    if (vnet.enabled && dev->vnet_port >= 0)
        vnet_tx_frame(dev->vnet_port, frame, data_len);
    s->regs[W6300_Sn_TX_RD0] = s->regs[W6300_Sn_TX_WR0];
    s->regs[W6300_Sn_TX_RD0 + 1] = s->regs[W6300_Sn_TX_WR0 + 1];
    s->regs[W6300_Sn_TX_FSR0] = (W6300_TX_BUF_SIZE >> 8) & 0xFF;
    s->regs[W6300_Sn_TX_FSR0 + 1] = W6300_TX_BUF_SIZE & 0xFF;
    s->regs[W6300_Sn_IR] |= W6300_IR_SENDOK;
    s->tx_dirty_valid = 0;
    s->tx_dirty_len = 0;
}

static int w6300_mode_is_udp(uint8_t mode) {
    return mode == W6300_MR_UDP || mode == W6300_MR_UDP6 || mode == W6300_MR_UDPD;
}

static int w6300_mode_is_tcp(uint8_t mode) {
    return mode == W6300_MR_TCP || mode == W6300_MR_TCP6 || mode == W6300_MR_TCPD;
}

static int w6300_mode_is_ipraw(uint8_t mode) {
    return mode == W6300_MR_IPRAW || mode == W6300_MR_IPRAW6;
}

/* True when this socket needs an IPv6 host path (pure-v6 modes).
 * Dual-stack TCPD/UDPD pick the family per-transaction (DIP6R vs DIPR),
 * so OPEN/CONNECT create AF_INET by default and upgrade on demand. */
static int w6300_mode_is_v6(uint8_t mode) {
    return mode == W6300_MR_TCP6 || mode == W6300_MR_UDP6 ||
           mode == W6300_MR_IPRAW6;
}

static int w6300_mode_is_dual(uint8_t mode) {
    return mode == W6300_MR_TCPD || mode == W6300_MR_UDPD;
}

/* Dual-stack destination select: nonzero DIP6R (16B) means IPv6 wins,
 * else IPv4. Matches ioLibrary semantics where TCPD/UDPD sockets carry
 * both DIPR and DIP6R and the stack picks by family. */
static int w6300_sock_dest_is_v6(w6300_socket_t *s) {
    for (int i = 0; i < 16; i++)
        if (s->regs[W6300_Sn_DIP6R0 + i]) return 1;
    return 0;
}

static void w6300_build_addr6(w6300_socket_t *s, struct sockaddr_in6 *addr) {
    memset(addr, 0, sizeof(*addr));
    addr->sin6_family = AF_INET6;
    for (int i = 0; i < 16; i++)
        addr->sin6_addr.s6_addr[i] = s->regs[W6300_Sn_DIP6R0 + i];
    addr->sin6_port = htons((uint16_t)(((uint16_t)s->regs[W6300_Sn_DPORTR0] << 8) |
                                       s->regs[W6300_Sn_DPORTR0 + 1]));
}

/* ESR mirrors the live transport: TCPM=IPv6 TCP, TCPOP=client (we only
 * ever open client sockets), IP6T=GUA when the remote is not link-local
 * (fe80::/10), else LLA. UDP/IPRAW leave ESR 0 (ioLibrary: valid on TCP
 * only). */
static void w6300_sock_update_esr(w6300_socket_t *s, int is_v6, int is_client) {
    uint8_t esr = 0;
    if (is_v6) {
        esr |= W6300_ESR_TCPM;
        if (is_client) esr |= W6300_ESR_TCPOP;
        if ((s->regs[W6300_Sn_DIP6R0] & 0xFF) != 0xFE ||
            (s->regs[W6300_Sn_DIP6R0 + 1] & 0xC0) != 0x80)
            esr |= W6300_ESR_IP6T;
    } else if (is_client) {
        esr |= W6300_ESR_TCPOP;
    }
    s->regs[W6300_Sn_ESR] = esr;
}

static void w6300_process_socket_cmd(w6300_t *dev, int sock) {
    w6300_socket_t *s = &dev->sockets[sock];
    uint8_t cmd = s->regs[W6300_Sn_CR];
    uint8_t mode = s->regs[W6300_Sn_MR];

    if (cmd == 0) return;

    switch (cmd) {
    case W6300_CMD_OPEN:
        /* ioLibrary: Sn_RTR/Sn_RCR seed from RTR/RCR when 0 at OPEN. */
        if (s->regs[W6300_Sn_RTR0] == 0 && s->regs[W6300_Sn_RTR0 + 1] == 0) {
            s->regs[W6300_Sn_RTR0] = dev->common[W6300_RTR0];
            s->regs[W6300_Sn_RTR0 + 1] = dev->common[W6300_RTR0 + 1];
        }
        if (s->regs[W6300_Sn_RCR] == 0)
            s->regs[W6300_Sn_RCR] = dev->common[W6300_RCR];
        /* ESR is live transport state: stale values must not survive
         * across OPEN (ioLibrary: valid on TCP only, set at CONNECT). */
        s->regs[W6300_Sn_ESR] = 0x00;
        s->retry_ticks = 0;
        /* Pure-v6 modes always dial AF_INET6; pure-v4 stay AF_INET.
         * Dual-stack TCPD/UDPD open AF_INET by default and upgrade to
         * AF_INET6 on CONNECT/CONNECT6 with a v6 destination, or speak
         * either family per-SEND for UDP. (Linux has no dual AF_INET
         * socket: the upgrade closes + recreates the fd.) */
        if (w6300_mode_is_tcp(mode)) {
            s->regs[W6300_Sn_SR] = W6300_SOCK_INIT;
            if (dev->live) {
                w6300_close_host_sock(s);
                if (w6300_mode_is_v6(mode)) {
                    s->host_fd = socket(AF_INET6, SOCK_STREAM, 0);
                } else {
                    s->host_fd = socket(AF_INET, SOCK_STREAM, 0);
                }
                if (s->host_fd >= 0) set_sock_nonblock(s->host_fd);
                w6300_sock_apply_ttl_tos(s);
            }
        } else if (w6300_mode_is_udp(mode)) {
            s->regs[W6300_Sn_SR] = W6300_SOCK_UDP;
            if (dev->live) {
                w6300_close_host_sock(s);
                if (w6300_mode_is_v6(mode)) {
                    s->host_fd = socket(AF_INET6, SOCK_DGRAM, 0);
                } else {
                    s->host_fd = socket(AF_INET, SOCK_DGRAM, 0);
                }
                if (s->host_fd >= 0) {
                    set_sock_nonblock(s->host_fd);
                    w6300_sock_apply_ttl_tos(s);
                    uint16_t src_port = ((uint16_t)s->regs[W6300_Sn_PORTR0] << 8) |
                                        s->regs[W6300_Sn_PORTR0 + 1];
                    if (src_port > 0) {
                        if (w6300_mode_is_v6(mode)) {
                            struct sockaddr_in6 bind6;
                            memset(&bind6, 0, sizeof(bind6));
                            bind6.sin6_family = AF_INET6;
                            bind6.sin6_addr = in6addr_any;
                            bind6.sin6_port = htons(src_port);
                            int opt = 1;
                            setsockopt(s->host_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
                            bind(s->host_fd, (struct sockaddr *)&bind6, sizeof(bind6));
                        } else {
                            struct sockaddr_in bind_addr;
                            memset(&bind_addr, 0, sizeof(bind_addr));
                            bind_addr.sin_family = AF_INET;
                            bind_addr.sin_addr.s_addr = INADDR_ANY;
                            bind_addr.sin_port = htons(src_port);
                            int opt = 1;
                            setsockopt(s->host_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
                            bind(s->host_fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
                        }
                    }
                }
            }
        } else if (mode == W6300_MR_MACRAW) {
            s->regs[W6300_Sn_SR] = W6300_SOCK_MACRAW;
            if (sock == 0)
                w6300_macraw_attach(dev, sock);
        } else if (w6300_mode_is_ipraw(mode)) {
            /* IPRAW4/6: raw IP socket. Status mirrors the family so
             * guests can tell them apart (ioLibrary SOCK_IPRAW4=0x32,
             * SOCK_IPRAW6=0x33). Live host path dials in SEND paths. */
            s->regs[W6300_Sn_SR] = (mode == W6300_MR_IPRAW6) ?
                W6300_SOCK_IPRAW6 : W6300_SOCK_IPRAW4;
            if (dev->live) {
                w6300_close_host_sock(s);
                if (mode == W6300_MR_IPRAW6)
                    s->host_fd = socket(AF_INET6, SOCK_RAW,
                        s->regs[W6300_Sn_PNR] ? s->regs[W6300_Sn_PNR] : IPPROTO_RAW);
                else
                    s->host_fd = socket(AF_INET, SOCK_RAW,
                        s->regs[W6300_Sn_PNR] ? s->regs[W6300_Sn_PNR] : IPPROTO_RAW);
                if (s->host_fd >= 0) set_sock_nonblock(s->host_fd);
            }
        } else if (mode == W6300_MR_CLOSE) {
            s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
        }
        s->regs[W6300_Sn_TX_FSR0] = (W6300_TX_BUF_SIZE >> 8) & 0xFF;
        s->regs[W6300_Sn_TX_FSR0 + 1] = W6300_TX_BUF_SIZE & 0xFF;
        s->tx_dirty_valid = 0;
        s->tx_dirty_len = 0;
        if (mode == W6300_MR_MACRAW) {
            s->rx_base = 0;
            s->regs[W6300_Sn_RX_RD0] = 0;
            s->regs[W6300_Sn_RX_RD0 + 1] = 0;
            s->regs[W6300_Sn_RX_WR0] = 0;
            s->regs[W6300_Sn_RX_WR0 + 1] = 0;
            s->regs[W6300_Sn_RX_RSR0] = 0;
            s->regs[W6300_Sn_RX_RSR0 + 1] = 0;
        }
        break;

    case W6300_CMD_LISTEN:
        if (s->regs[W6300_Sn_SR] == W6300_SOCK_INIT) {
            s->regs[W6300_Sn_SR] = W6300_SOCK_LISTEN;
            /* Dual-stack TCPD LISTEN follows ioLibrary: v6 dest regs
             * set => IPv6 server. Upgrade the AF_INET fd from OPEN. */
            int listen_v6 = w6300_mode_is_v6(mode) ||
                (w6300_mode_is_dual(mode) &&
                 w6300_mode_is_tcp(mode) && w6300_sock_dest_is_v6(s));
            if (dev->live && s->host_fd >= 0) {
                if (listen_v6 && !w6300_sock_host_v6(s)) {
                    w6300_sock_recreate(&s->host_fd, AF_INET6, SOCK_STREAM, 0);
                }
                /* IPv6 LISTEN: the OPEN path created AF_INET6 for v6
                 * modes; bind ::/port the same way. */
                if (listen_v6) {
                    struct sockaddr_in6 bind6;
                    memset(&bind6, 0, sizeof(bind6));
                    bind6.sin6_family = AF_INET6;
                    bind6.sin6_addr = in6addr_any;
                    bind6.sin6_port = htons((uint16_t)(((uint16_t)s->regs[W6300_Sn_PORTR0] << 8) |
                                                      s->regs[W6300_Sn_PORTR0 + 1]));
                    int opt = 1;
                    setsockopt(s->host_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
                    if (bind(s->host_fd, (struct sockaddr *)&bind6, sizeof(bind6)) == 0 &&
                        listen(s->host_fd, 1) == 0) {
                        s->host_listen_fd = s->host_fd;
                        s->host_fd = -1;
                    }
                } else {
                    uint16_t src_port = ((uint16_t)s->regs[W6300_Sn_PORTR0] << 8) |
                                        s->regs[W6300_Sn_PORTR0 + 1];
                    struct sockaddr_in bind_addr;
                    memset(&bind_addr, 0, sizeof(bind_addr));
                    bind_addr.sin_family = AF_INET;
                    bind_addr.sin_addr.s_addr = INADDR_ANY;
                    bind_addr.sin_port = htons(src_port);
                    int opt = 1;
                    setsockopt(s->host_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
                    if (bind(s->host_fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) == 0 &&
                        listen(s->host_fd, 1) == 0) {
                        s->host_listen_fd = s->host_fd;
                        s->host_fd = -1;
                    }
                }
            }
#ifdef __EMSCRIPTEN__
            if (dev->live) {
                int wport = ((int)s->regs[W6300_Sn_PORTR0] << 8) | (int)s->regs[W6300_Sn_PORTR0 + 1];
                if (listen_v6) {
                    /* LISTEN6: [0x4C, sock, port_lo, port_hi, '6'] (5B).
                     * v4 proxies parse the first 4 bytes and ignore the
                     * flag; updated net_proxy.py binds AF_INET6. */
                    uint8_t msg[5];
                    msg[0] = 0x4C; msg[1] = (uint8_t)(sock & 0xFF);
                    msg[2] = (uint8_t)(wport & 0xFF); msg[3] = (uint8_t)((wport >> 8) & 0xFF);
                    msg[4] = 0x36; /* '6' */
                    w6300_ws_tx_push(msg, 5);
                } else {
                    uint8_t msg[4];
                    msg[0] = 0x4C; msg[1] = (uint8_t)(sock & 0xFF);
                    msg[2] = (uint8_t)(wport & 0xFF); msg[3] = (uint8_t)((wport >> 8) & 0xFF);
                    w6300_ws_tx_push(msg, 4);
                }
            }
#endif
        }
        break;

    case W6300_CMD_CONNECT:
    case W6300_CMD_CONNECT6:
    {
        int want_v6 = (cmd == W6300_CMD_CONNECT6) ||
                      (w6300_mode_is_v6(mode) && !w6300_mode_is_dual(mode));
        /* Dual-stack CONNECT (0x04) with a v6 destination upgrades to
         * IPv6; pure-v4 modes (TCP/UDP/IPRAW4) always stay IPv4. */
        if (cmd == W6300_CMD_CONNECT && w6300_mode_is_dual(mode) &&
            w6300_sock_dest_is_v6(s))
            want_v6 = 1;
        if (s->regs[W6300_Sn_SR] == W6300_SOCK_INIT) {
            s->retry_ticks = 0;
            if (dev->live && s->host_fd >= 0) {
                /* Dual-stack upgrade: a TCPD/UDPD socket opened AF_INET
                 * must become AF_INET6 before dialling a v6 address
                 * (connect/sendto on the wrong family is EAFNOSUPPORT).
                 * Close + recreate; the guest-visible Sn_SR/IR flow is
                 * unchanged. Pure-v6 modes already own an AF_INET6 fd;
                 * pure-v4 never upgrades. */
                if (want_v6 && w6300_mode_is_dual(mode) &&
                    !w6300_sock_host_v6(s)) {
                    int st = w6300_mode_is_udp(mode) ? SOCK_DGRAM :
                             (w6300_mode_is_ipraw(mode) ? SOCK_RAW : SOCK_STREAM);
                    int proto = 0;
                    if (w6300_mode_is_ipraw(mode))
                        proto = s->regs[W6300_Sn_PNR] ? s->regs[W6300_Sn_PNR] : IPPROTO_RAW;
                    if (w6300_sock_recreate(&s->host_fd, AF_INET6, st, proto) < 0) {
                        s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
                        s->regs[W6300_Sn_IR] |= W6300_IR_TIMEOUT;
                        break;
                    }
                    w6300_sock_apply_ttl_tos(s);
                }
                if (want_v6 && !w6300_host_fd_is_v6(s->host_fd)) {
                    /* Defensive: never dial v6 on an fd the kernel
                     * reports as non-v6 (fd reuse / platform quirk). */
                    fprintf(stderr, "[W6300] Socket %d v6 dial on non-v6 fd\n", sock);
                    s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
                    s->regs[W6300_Sn_IR] |= W6300_IR_TIMEOUT;
                    break;
                }
                if (want_v6) {
                    struct sockaddr_in6 dest6;
                    w6300_build_addr6(s, &dest6);
                    int rc = connect(s->host_fd, (struct sockaddr *)&dest6, sizeof(dest6));
                    if (rc == 0) {
                        s->regs[W6300_Sn_SR] = W6300_SOCK_ESTABLISHED;
                        w6300_sock_update_esr(s, 1, 1);
                    } else if (errno == EINPROGRESS) {
                        s->regs[W6300_Sn_SR] = W6300_SOCK_SYNSENT;
                        w6300_sock_update_esr(s, 1, 1);
                    } else {
                        fprintf(stderr, "[W6300] Socket %d connect6 failed: %s\n",
                                sock, strerror(errno));
                        s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
                    }
                } else {
                    struct sockaddr_in dest;
                    w6300_build_addr(s, &dest);
                    int rc = connect(s->host_fd, (struct sockaddr *)&dest, sizeof(dest));
                    if (rc == 0) {
                        s->regs[W6300_Sn_SR] = W6300_SOCK_ESTABLISHED;
                        w6300_sock_update_esr(s, 0, 1);
                    } else if (errno == EINPROGRESS) {
                        s->regs[W6300_Sn_SR] = W6300_SOCK_SYNSENT;
                        w6300_sock_update_esr(s, 0, 1);
                    } else {
                        fprintf(stderr, "[W6300] Socket %d connect failed: %s\n",
                                sock, strerror(errno));
                        s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
                    }
                }
            } else {
                s->regs[W6300_Sn_SR] = W6300_SOCK_ESTABLISHED;
                w6300_sock_update_esr(s, want_v6, 1);
            }
#ifdef __EMSCRIPTEN__
            if (dev->live) {
                if (want_v6) {
                    /* CONNECT6: [0x43, sock, 18, 0, udp, 16B ip6, port_lo, port_hi].
                     * Same 0x43 family as v4 so older proxies ignore the
                     * length they don't know; updated net_proxy.py parses
                     * both (len 11 = v4, len 21 = v6). */
                    uint8_t msg[23];
                    msg[0] = 0x43; msg[1] = (uint8_t)(sock & 0xFF);
                    msg[2] = 18; msg[3] = 0;
                    msg[4] = w6300_mode_is_udp(mode) ? 1 : 0;
                    for (int i = 0; i < 16; i++)
                        msg[5 + i] = s->regs[W6300_Sn_DIP6R0 + i];
                    {
                        int wport = ((int)s->regs[W6300_Sn_DPORTR0] << 8) |
                                    (int)s->regs[W6300_Sn_DPORTR0 + 1];
                        msg[21] = (uint8_t)(wport & 0xFF);
                        msg[22] = (uint8_t)((wport >> 8) & 0xFF);
                    }
                    w6300_ws_tx_push(msg, 23);
                } else {
                    uint8_t msg[11];
                    msg[0] = 0x43; msg[1] = (uint8_t)(sock & 0xFF);
                    msg[2] = 6; msg[3] = 0;
                    msg[4] = w6300_mode_is_udp(mode) ? 1 : 0;
                    msg[5] = s->regs[W6300_Sn_DIPR0];
                    msg[6] = s->regs[W6300_Sn_DIPR0 + 1];
                    msg[7] = s->regs[W6300_Sn_DIPR0 + 2];
                    msg[8] = s->regs[W6300_Sn_DIPR0 + 3];
                    {
                        int wport = ((int)s->regs[W6300_Sn_DPORTR0] << 8) |
                                    (int)s->regs[W6300_Sn_DPORTR0 + 1];
                        msg[9] = (uint8_t)(wport & 0xFF);
                        msg[10] = (uint8_t)((wport >> 8) & 0xFF);
                    }
                    w6300_ws_tx_push(msg, 11);
                }
            }
#endif
        } /* if (INIT) */
    } /* CONNECT/CONNECT6 */
        break;

    case W6300_CMD_DISCON:
        if (s->regs[W6300_Sn_SR] == W6300_SOCK_ESTABLISHED) {
            s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
            s->retry_ticks = 0;
            if (dev->live) w6300_close_host_sock(s);
            s->regs[W6300_Sn_IR] |= W6300_IR_DISCON;
        }
        break;

    case W6300_CMD_CLOSE:
        s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
        s->retry_ticks = 0;
        if (dev->live) w6300_close_host_sock(s);
#ifdef __EMSCRIPTEN__
        if (dev->live) {
            uint8_t msg[2];
            msg[0] = 0x58; msg[1] = (uint8_t)(sock & 0xFF);
            w6300_ws_tx_push(msg, 2);
        }
#endif
        break;

    case W6300_CMD_SEND:
    case W6300_CMD_SEND_MAC:
    case W6300_CMD_SEND6:
    {
        /* SEND_MAC (0x21, UDP only per ioLibrary): same as SEND but
         * uses Sn_DHAR directly, skipping ARP. The model has no ARP
         * table to skip (live UDP always sendto()s the programmed
         * DIPR), so SEND_MAC shares the SEND datapath exactly. */
        /* SEND6 targets IPv6 explicitly; plain SEND on a dual socket
         * follows the destination (DIP6R set => v6); pure-v6 modes are
         * always v6. Pure-v4 stays v4. */
        int is_v6 = (cmd == W6300_CMD_SEND6) ||
                    (w6300_mode_is_v6(mode) && !w6300_mode_is_dual(mode));
        if (cmd == W6300_CMD_SEND && w6300_mode_is_dual(mode) &&
            w6300_sock_dest_is_v6(s))
            is_v6 = 1;
        uint16_t tx_rd = ((uint16_t)s->regs[W6300_Sn_TX_RD0] << 8) |
                         s->regs[W6300_Sn_TX_RD0 + 1];
        uint16_t tx_wr = ((uint16_t)s->regs[W6300_Sn_TX_WR0] << 8) |
                         s->regs[W6300_Sn_TX_WR0 + 1];
        uint16_t data_len = (uint16_t)(tx_wr - tx_rd);
        if (data_len > W6300_TX_BUF_SIZE) data_len = W6300_TX_BUF_SIZE;

        if (mode == W6300_MR_MACRAW && sock == 0 && dev->vnet_port >= 0) {
            w6300_macraw_send(dev, sock);
            break;
        }

        if (dev->live && s->host_fd >= 0 && data_len > 0) {
            uint8_t send_buf[W6300_TX_BUF_SIZE];
            for (uint16_t i = 0; i < data_len; i++) {
                send_buf[i] = s->tx_buf[(tx_rd + i) % W6300_TX_BUF_SIZE];
            }
            /* Dual-stack UDP upgrade: an AF_INET UDPD socket sending to
             * a v6 destination becomes AF_INET6 first (same recreate as
             * CONNECT). TCP dual uses the CONNECT-time upgrade; a SEND
             * that still finds the wrong family errors TIMEOUT. */
            if (is_v6 && w6300_mode_is_udp(mode) &&
                w6300_mode_is_dual(mode) && !w6300_sock_host_v6(s)) {
                w6300_sock_recreate(&s->host_fd, AF_INET6, SOCK_DGRAM, 0);
                w6300_sock_apply_ttl_tos(s);
            }
            if (is_v6 && !w6300_host_fd_is_v6(s->host_fd)) {
                s->regs[W6300_Sn_IR] |= W6300_IR_TIMEOUT;
            } else if (is_v6 || (w6300_mode_is_udp(mode) &&
                           w6300_mode_is_dual(mode) &&
                           w6300_sock_dest_is_v6(s))) {
                struct sockaddr_in6 dest6;
                w6300_build_addr6(s, &dest6);
                sendto(s->host_fd, send_buf, data_len, 0,
                       (struct sockaddr *)&dest6, sizeof(dest6));
            } else if (w6300_mode_is_udp(mode)) {
                struct sockaddr_in dest;
                w6300_build_addr(s, &dest);
                sendto(s->host_fd, send_buf, data_len, 0,
                       (struct sockaddr *)&dest, sizeof(dest));
            } else if (w6300_mode_is_ipraw(mode)) {
                /* IPRAW: raw IP datagram, protocol from Sn_PNR. Host
                 * raw sockets need privilege; attempt and surface
                 * TIMEOUT on failure like silicon would. */
                if (is_v6) {
                    struct sockaddr_in6 dest6;
                    w6300_build_addr6(s, &dest6);
                    if (sendto(s->host_fd, send_buf, data_len, 0,
                               (struct sockaddr *)&dest6,
                               sizeof(dest6)) < 0)
                        s->regs[W6300_Sn_IR] |= W6300_IR_TIMEOUT;
                } else {
                    struct sockaddr_in dest;
                    w6300_build_addr(s, &dest);
                    if (sendto(s->host_fd, send_buf, data_len, 0,
                               (struct sockaddr *)&dest,
                               sizeof(dest)) < 0)
                        s->regs[W6300_Sn_IR] |= W6300_IR_TIMEOUT;
                }
            } else {
                send(s->host_fd, send_buf, data_len, MSG_NOSIGNAL);
            }
        }
#ifdef __EMSCRIPTEN__
        if (dev->live && data_len > 0) {
            if (is_v6) {
                /* SEND6: [0x57, sock, len_lo, len_hi, '6',
                 * 16B ip6, port_lo, port_hi, payload...] (23B header).
                 * Updated net_proxy.py parses flag '6' + 18B v6 header;
                 * old proxies only understand the 4B header, so gate
                 * this framing on __EMSCRIPTEN__ (native never emits
                 * it). */
                uint8_t hdr[23];
                hdr[0] = 0x57; hdr[1] = (uint8_t)(sock & 0xFF);
                hdr[2] = (uint8_t)(data_len & 0xFF);
                hdr[3] = (uint8_t)((data_len >> 8) & 0xFF);
                hdr[4] = 0x36; /* '6': IPv6 destination follows */
                for (int i = 0; i < 16; i++)
                    hdr[5 + i] = s->regs[W6300_Sn_DIP6R0 + i];
                {
                    int wport = ((int)s->regs[W6300_Sn_DPORTR0] << 8) |
                                (int)s->regs[W6300_Sn_DPORTR0 + 1];
                    hdr[21] = (uint8_t)(wport & 0xFF);
                    hdr[22] = (uint8_t)((wport >> 8) & 0xFF);
                }
                w6300_ws_tx_push(hdr, 23);
                for (uint16_t i = 0; i < data_len; i++) {
                    uint8_t b = s->tx_buf[(tx_rd + i) % W6300_TX_BUF_SIZE];
                    w6300_ws_tx_push(&b, 1);
                }
            } else {
                uint8_t hdr[4];
                hdr[0] = 0x57; hdr[1] = (uint8_t)(sock & 0xFF);
                hdr[2] = (uint8_t)(data_len & 0xFF);
                hdr[3] = (uint8_t)((data_len >> 8) & 0xFF);
                w6300_ws_tx_push(hdr, 4);
                for (uint16_t i = 0; i < data_len; i++) {
                    uint8_t b = s->tx_buf[(tx_rd + i) % W6300_TX_BUF_SIZE];
                    w6300_ws_tx_push(&b, 1);
                }
            }
        }
#endif
        s->regs[W6300_Sn_TX_RD0] = s->regs[W6300_Sn_TX_WR0];
        s->regs[W6300_Sn_TX_RD0 + 1] = s->regs[W6300_Sn_TX_WR0 + 1];
        s->regs[W6300_Sn_TX_FSR0] = (W6300_TX_BUF_SIZE >> 8) & 0xFF;
        s->regs[W6300_Sn_TX_FSR0 + 1] = W6300_TX_BUF_SIZE & 0xFF;
        s->regs[W6300_Sn_IR] |= W6300_IR_SENDOK;
        break;
    }

    case W6300_CMD_SEND_KEEP:
    {
        /* Keep-alive: 1-byte probe on an ESTABLISHED TCP socket.
         * Live path sends a single 0x00 byte (kernel KA would need
         * SO_KEEPALIVE tuning; this matches W5500-equivalent behavior
         * of surfacing SENDOK). Offline: just flag SENDOK. */
        if (dev->live && s->host_fd >= 0 &&
            s->regs[W6300_Sn_SR] == W6300_SOCK_ESTABLISHED) {
            uint8_t probe = 0x00;
            if (w6300_mode_is_v6(mode)) {
                struct sockaddr_in6 peer6;
                socklen_t plen = sizeof(peer6);
                if (getpeername(s->host_fd, (struct sockaddr *)&peer6,
                                &plen) == 0)
                    sendto(s->host_fd, &probe, 1, 0,
                           (struct sockaddr *)&peer6, plen);
                else
                    send(s->host_fd, &probe, 1, MSG_NOSIGNAL);
            } else {
                send(s->host_fd, &probe, 1, MSG_NOSIGNAL);
            }
        }
        s->regs[W6300_Sn_IR] |= W6300_IR_SENDOK;
        break;
    }

    case W6300_CMD_RECV:
        if (mode == W6300_MR_MACRAW && sock == 0 && dev->vnet_port >= 0) {
            uint16_t rx_rd = ((uint16_t)s->regs[W6300_Sn_RX_RD0] << 8) |
                             s->regs[W6300_Sn_RX_RD0 + 1];
            uint16_t rx_rsr = ((uint16_t)s->regs[W6300_Sn_RX_RSR0] << 8) |
                              s->regs[W6300_Sn_RX_RSR0 + 1];
            W6300_TR("RECV MACRAW rd=0x%04X rsr=%u base=0x%04X cursor=%s0x%04X",
                     rx_rd, rx_rsr, s->rx_base,
                     s->rx_cursor_valid ? "" : "(invalid)",
                     s->rx_cursor_valid ? s->rx_cursor_base : 0);
            /* Invalidate the VDM cursor: the next RX-buffer read frame
             * carries a fresh address phase and must latch a fresh
             * cursor. Without this, the second frame of a two-frame
             * consume (header frame, then payload frame) reuses the
             * first frame's cursor and serves payload shifted by the
             * header length (OFFER payload corrupted -> no REQUEST). */
            s->rx_cursor_valid = 0;
            if (rx_rd != s->rx_base && rx_rsr > 0) {
                uint16_t pulled = (uint16_t)(rx_rd - s->rx_base);
                if (pulled > rx_rsr) pulled = rx_rsr;
                uint16_t remain = (uint16_t)(rx_rsr - pulled);
                s->rx_base = rx_rd;
                uint16_t tail = (uint16_t)(rx_rd + remain);
                s->regs[W6300_Sn_RX_WR0]     = (tail >> 8) & 0xFF;
                s->regs[W6300_Sn_RX_WR0 + 1] = tail & 0xFF;
                s->regs[W6300_Sn_RX_RSR0]     = (remain >> 8) & 0xFF;
                s->regs[W6300_Sn_RX_RSR0 + 1] = remain & 0xFF;
                if (remain == 0)
                    s->regs[W6300_Sn_IR] &= (uint8_t)~W6300_IR_RECV;
                break;
            }
            if (rx_rsr == 0) break;
            if (rx_rsr >= 2) {
                uint16_t base = s->rx_base % W6300_RX_BUF_SIZE;
                uint16_t flen = ((uint16_t)s->rx_buf[base] << 8) |
                                s->rx_buf[(base + 1) % W6300_RX_BUF_SIZE];
                uint16_t total = flen;
                if (total > rx_rsr) total = rx_rsr;
                uint16_t remain = (uint16_t)(rx_rsr - total);
                uint16_t new_rd = (uint16_t)(s->rx_base + total);
                s->rx_base = new_rd;
                uint16_t tail = (uint16_t)(new_rd + remain);
                s->regs[W6300_Sn_RX_RD0]     = (new_rd >> 8) & 0xFF;
                s->regs[W6300_Sn_RX_RD0 + 1] = new_rd & 0xFF;
                s->regs[W6300_Sn_RX_WR0]     = (tail >> 8) & 0xFF;
                s->regs[W6300_Sn_RX_WR0 + 1] = tail & 0xFF;
                s->regs[W6300_Sn_RX_RSR0]     = (remain >> 8) & 0xFF;
                s->regs[W6300_Sn_RX_RSR0 + 1] = remain & 0xFF;
                if (remain == 0)
                    s->regs[W6300_Sn_IR] &= (uint8_t)~W6300_IR_RECV;
            } else {
                s->regs[W6300_Sn_RX_RSR0] = 0;
                s->regs[W6300_Sn_RX_RSR0 + 1] = 0;
                s->regs[W6300_Sn_IR] &= (uint8_t)~W6300_IR_RECV;
            }
            break;
        }
        s->regs[W6300_Sn_RX_RSR0] = 0;
        s->regs[W6300_Sn_RX_RSR0 + 1] = 0;
        break;

    default:
        break;
    }

    s->regs[W6300_Sn_CR] = 0x00;
    w6300_board_refresh_int();
}

/* ========================================================================
 * Register read/write
 * ======================================================================== */

static uint16_t w6300_tx_buf_kb(w6300_t *dev, int sock) {
    uint32_t kb = dev->sockets[sock].regs[W6300_Sn_TX_BSR];
    if (kb == 0) kb = 4;
    if (kb > 16) kb = 16;
    return (uint16_t)kb;
}

static uint8_t w6300_sysr(w6300_t *dev) {
    uint8_t v = W6300_SYSR_SPI;
    if (dev->chip_locked) v |= W6300_SYSR_CHPL;
    if (dev->net_locked)  v |= W6300_SYSR_NETL;
    if (dev->phy_locked)  v |= W6300_SYSR_PHYL;
    return v;
}

static uint8_t w6300_physr(void) {
    /* Link up, 100M full-duplex, auto mode, cable on. Note opposite
     * polarity vs W5500: SPD=0 means 100M, DPX=0 means full. */
    return W6300_PHYSR_LNK;
}

static uint8_t w6300_read_common(w6300_t *dev, uint16_t addr) {
    switch (addr) {
    case W6300_CIDR0: return 0x61;  /* == VERSIONR (Arduino alias 0x0000) */
    case W6300_CIDR1: return 0x00;
    case W6300_CIDR2: return 0x11;
    case W6300_SYSR:  return w6300_sysr(dev);
    case W6300_PHYSR: return w6300_physr();
    case W6300_SIR: {
        /* Masked: a socket bit shows only when its Sn_IR has a bit
         * the guest enabled in Sn_IMR AND the socket is enabled in
         * SIMR. (Global IEN gates only the INTn pin, not the SIR
         * register itself — ioLibrary polls SIR with IEN=0.) */
        uint8_t sir = 0;
        for (int i = 0; i < W6300_NUM_SOCKETS; i++) {
            uint8_t ir = dev->sockets[i].regs[W6300_Sn_IR];
            uint8_t imr = dev->sockets[i].regs[W6300_Sn_IMR];
            if ((ir & imr) == 0) continue;
            if ((dev->common[W6300_SIMR] & (1u << i)) == 0) continue;
            sir |= (uint8_t)(1u << i);
        }
        return sir;
    }
    default:
        if (addr < sizeof(dev->common))
            return dev->common[addr];
        return 0x00;
    }
}

static void w6300_write_common(w6300_t *dev, uint16_t addr, uint8_t val) {
    switch (addr) {
    case W6300_CIDR0:
    case W6300_CIDR1:
    case W6300_CIDR2:
    case W6300_SYSR:
    case W6300_PHYSR:
    case W6300_SIR:
        return;  /* read-only (CIDR0 doubles as Arduino VERSIONR) */
    case W6300_SYCR0:
        if (dev->chip_locked) return;
        if (val & W6300_SYCR0_RST) {
            int live = dev->live;
            w6300_init(dev);
            dev->live = live;
        }
        return;
    case W6300_SYCR1:
        /* IEN writable always; CLKSEL needs chip-unlock */
        if (dev->chip_locked)
            val = (uint8_t)((dev->common[addr] & 0x01) | (val & 0x80));
        if (addr < sizeof(dev->common)) dev->common[addr] = val;
        return;
    case W6300_PHYCR0:
        if (dev->phy_locked) return;
        if (addr < sizeof(dev->common)) dev->common[addr] = val;
        return;
    case W6300_PHYCR1:
        if (dev->phy_locked) return;
        if (addr < sizeof(dev->common)) dev->common[addr] = val;
        return;
    case W6300_CHPLCKR:
        dev->chip_locked = (val == W6300_CHIP_UNLOCK) ? 0 : 1;
        return;
    case W6300_NETLCKR:
        dev->net_locked = (val == W6300_NET_UNLOCK) ? 0 : 1;
        return;
    case W6300_PHYLCKR:
        dev->phy_locked = (val == W6300_PHY_UNLOCK) ? 0 : 1;
        return;
    case W6300_IRCLR:
        dev->common[W6300_IR] &= (uint8_t)~val;
        w6300_board_refresh_int();
        return;
    case W6300_SLIRCLR:
        dev->common[W6300_SLIR] &= (uint8_t)~val;
        w6300_board_refresh_int();
        return;
    case W6300_SLCR:
        /* Socket-less commands complete instantly in the model. */
        dev->common[W6300_SLCR] = 0x00;
        if (val & W6300_SLCR_ARP4)  dev->common[W6300_SLIR] |= W6300_SLIR_ARP4;
        if (val & W6300_SLCR_PING4) dev->common[W6300_SLIR] |= W6300_SLIR_PING4;
        if (val & W6300_SLCR_ARP6)  dev->common[W6300_SLIR] |= W6300_SLIR_ARP6;
        if (val & W6300_SLCR_PING6) dev->common[W6300_SLIR] |= W6300_SLIR_PING6;
        if (val & W6300_SLCR_NS)    dev->common[W6300_SLIR] |= W6300_SLIR_NS;
        if (val & W6300_SLCR_RS)    dev->common[W6300_SLIR] |= W6300_SLIR_RS;
        if (val & W6300_SLCR_UNA) {
            /* UNA (neighbor Unreachability detection): silicon sends NS
             * and reports via NS; model completes instantly the same. */
            dev->common[W6300_SLIR] |= W6300_SLIR_NS;
        }
        w6300_board_refresh_int();
        return;
    case W6300_TCNTRCLR:
        dev->common[W6300_TCNTR0] = 0x00;
        dev->common[W6300_TCNTR0 + 1] = 0x00;
        return;
    default:
        break;
    }
    /* Network registers need NET-unlock: IPv4 SHAR/GAR/SUBR/SIPR plus
     * the IPv6 block LLAR/GUAR/SUB6R (ioLibrary: NETLOCK covers GAR,
     * SUBR, SHAR, SIPR, LLAR, GUAR, SUB6R). GA6R, the socket-less
     * destination/ping registers and the ICMP unreach latches are
     * outside the lock mechanism (GA6R explicitly excluded). */
    if ((addr >= W6300_SHAR0 && addr < W6300_SHAR0 + 6) ||
        (addr >= W6300_GAR0 && addr < W6300_GAR0 + 4) ||
        (addr >= W6300_SUBR0 && addr < W6300_SUBR0 + 4) ||
        (addr >= W6300_SIPR0 && addr < W6300_SIPR0 + 4) ||
        (addr >= W6300_LLAR0 && addr < W6300_LLAR0 + 16) ||
        (addr >= W6300_GUAR0 && addr < W6300_GUAR0 + 16) ||
        (addr >= W6300_SUB6R0 && addr < W6300_SUB6R0 + 16)) {
        if (dev->net_locked) return;
    }
    /* Read-only network state: RA info, ICMP unreach latches, SLDHAR. */
    if ((addr >= W6300_SLDHAR0 && addr < W6300_SLDHAR0 + 6) ||
        (addr >= W6300_UIPR0 && addr < W6300_UIPR0 + 4) ||
        (addr >= W6300_UPORTR0 && addr < W6300_UPORTR0 + 2) ||
        (addr >= W6300_UIP6R0 && addr < W6300_UIP6R0 + 16) ||
        (addr >= W6300_UPORT6R0 && addr < W6300_UPORT6R0 + 2) ||
        (addr >= W6300_PLR && addr <= W6300_PAR0 + 15) ||
        addr == W6300_TCNTR0 || addr == W6300_TCNTR0 + 1) {
        return;
    }
    if (addr < sizeof(dev->common)) {
        dev->common[addr] = val;
        if (addr >= W6300_SHAR0 && addr < W6300_SHAR0 + 6 &&
            dev->vnet_port >= 0) {
            uint8_t mac[6];
            for (int i = 0; i < 6; i++)
                mac[i] = dev->common[W6300_SHAR0 + i];
            vnet_update_port_mac(dev->vnet_port, mac);
            for (int i = 0; i < w6300_macraw_ndevs; i++) {
                if (w6300_macraw_devs[i].dev == dev) {
                    memcpy(w6300_macraw_devs[i].mac, mac, 6);
                    break;
                }
            }
        }
    }
}

static uint8_t w6300_read_sock(w6300_t *dev, int sock, uint16_t addr) {
    w6300_socket_t *s = &dev->sockets[sock];
    if (addr == W6300_Sn_TX_FSR0 || addr == W6300_Sn_TX_FSR0 + 1) {
        uint16_t kb = w6300_tx_buf_kb(dev, sock);
        uint16_t tx_rd = ((uint16_t)s->regs[W6300_Sn_TX_RD0] << 8) |
                         s->regs[W6300_Sn_TX_RD0 + 1];
        uint16_t tx_wr = ((uint16_t)s->regs[W6300_Sn_TX_WR0] << 8) |
                         s->regs[W6300_Sn_TX_WR0 + 1];
        uint16_t used = (uint16_t)(tx_wr - tx_rd);
        uint32_t cap = (uint32_t)kb * 1024u;
        uint16_t free = used >= cap ? 0 : (uint16_t)(cap - used);
        return (addr == W6300_Sn_TX_FSR0) ? (free >> 8) & 0xFF : free & 0xFF;
    }
    if (addr < W6300_SOCKET_REG_SIZE)
        return s->regs[addr];
    return 0x00;
}

static void w6300_write_sock(w6300_t *dev, int sock, uint16_t addr, uint8_t val) {
    w6300_socket_t *s = &dev->sockets[sock];
    if (addr == W6300_Sn_SR) return;           /* read-only */
    if (addr == W6300_Sn_ESR) return;          /* read-only (TCP live state) */
    /* Sn_IR is write-1-to-clear on real silicon (the Arduino driver
     * uses setSn_IR(Sn_IR_RECV) at the top of every readFrameSize to
     * re-arm the RX flag; a plain store would be ignored... and the
     * old code IGNORED all Sn_IR writes, so the RECV bit stayed stuck
     * once the first frame arrived and readFrameSize never saw a fresh
     * edge). Clear the written-1 bits, like the Sn_IRCLR path. */
    if (addr == W6300_Sn_IR) {
        s->regs[W6300_Sn_IR] &= (uint8_t)~val;
        w6300_board_refresh_int();
        return;
    }
    if (addr == W6300_Sn_TX_FSR0 || addr == W6300_Sn_TX_FSR0 + 1) return;
    if (addr == W6300_Sn_TX_RD0 || addr == W6300_Sn_TX_RD0 + 1) return;
    if (addr == W6300_Sn_RX_RSR0 || addr == W6300_Sn_RX_RSR0 + 1) return;
    if (addr == W6300_Sn_RX_WR0 || addr == W6300_Sn_RX_WR0 + 1) return;
    if (addr == W6300_Sn_IRCLR) {
        s->regs[W6300_Sn_IR] &= (uint8_t)~val;
        w6300_board_refresh_int();
        return;
    }
    if (addr >= W6300_SOCKET_REG_SIZE) return;
    s->regs[addr] = val;
    if (addr == W6300_Sn_CR)
        w6300_process_socket_cmd(dev, sock);
}

static uint8_t w6300_read_byte(w6300_t *dev, uint8_t bsb, uint16_t addr) {
    int type = blk_type(bsb);
    int sock = blk_socket(bsb);
    switch (type) {
    case 0:
        return w6300_read_common(dev, addr);
    case 1:
        if (sock >= 0 && sock < W6300_NUM_SOCKETS)
            return w6300_read_sock(dev, sock, addr);
        return 0x00;
    case 2:
        if (sock >= 0 && sock < W6300_NUM_SOCKETS)
            return dev->sockets[sock].tx_buf[addr % W6300_TX_BUF_SIZE];
        return 0x00;
    case 3:
        if (sock >= 0 && sock < W6300_NUM_SOCKETS) {
            if (sock == 0 &&
                dev->sockets[sock].regs[W6300_Sn_MR] == W6300_MR_MACRAW &&
                dev->sockets[sock].regs[W6300_Sn_SR] == W6300_SOCK_MACRAW) {
                w6300_socket_t *ms = &dev->sockets[sock];
                /* First RX-buffer read of a CS frame latches the VDM
                 * cursor. The PL022 path latches in w6300_spi_xfer
                 * before calling here; the PIO-QSPI bridge calls here
                 * directly from its on-demand RXF serve, so latch here
                 * too (no-op when already latched). Without this the
                 * bridge serves rx_buf[rx_base+addr] instead of
                 * rx_buf[rx_base+(addr-cursor)]: correct while rx_base
                 * is 0, but every frame after the first RECV advances
                 * rx_base and the guest reads shifted by rx_base bytes
                 * (OFFER consumed corrupted -> no DHCP REQUEST). */
                if (!ms->rx_cursor_valid) {
                    ms->rx_cursor_base = addr;
                    ms->rx_cursor_valid = 1;
                }
                uint16_t base = ms->rx_base % W6300_RX_BUF_SIZE;
                uint16_t off = (uint16_t)(addr - ms->rx_cursor_base);
                return ms->rx_buf[(base + off) % W6300_RX_BUF_SIZE];
            }
            return dev->sockets[sock].rx_buf[addr % W6300_RX_BUF_SIZE];
        }
        return 0x00;
    }
    return 0x00;
}

static void w6300_write_byte(w6300_t *dev, uint8_t bsb, uint16_t addr,
                             uint8_t val) {
    int type = blk_type(bsb);
    int sock = blk_socket(bsb);
    switch (type) {
    case 0:
        w6300_write_common(dev, addr, val);
        break;
    case 1:
        if (sock >= 0 && sock < W6300_NUM_SOCKETS)
            w6300_write_sock(dev, sock, addr, val);
        break;
    case 2:
        if (sock >= 0 && sock < W6300_NUM_SOCKETS) {
            w6300_socket_t *ts = &dev->sockets[sock];
            ts->tx_buf[addr % W6300_TX_BUF_SIZE] = val;
            if (!ts->tx_dirty_valid) {
                ts->tx_dirty_base = addr % W6300_TX_BUF_SIZE;
                ts->tx_dirty_len = 1;
                ts->tx_dirty_valid = 1;
            } else {
                uint16_t end = (uint16_t)(ts->tx_dirty_base + ts->tx_dirty_len);
                uint16_t a = addr % W6300_TX_BUF_SIZE;
                if (a == (uint16_t)(end % W6300_TX_BUF_SIZE))
                    ts->tx_dirty_len++;
                else if (a == 0 && ts->tx_dirty_len > 0 &&
                         ts->tx_dirty_base != 0) {
                    ts->tx_dirty_base = 0;
                    ts->tx_dirty_len = 1;
                } else {
                    ts->tx_dirty_len = W6300_TX_BUF_SIZE;
                }
                if (ts->tx_dirty_len > W6300_TX_BUF_SIZE)
                    ts->tx_dirty_len = W6300_TX_BUF_SIZE;
            }
        }
        break;
    case 3:
        if (sock >= 0 && sock < W6300_NUM_SOCKETS) {
            if (sock == 0 &&
                dev->sockets[sock].regs[W6300_Sn_MR] == W6300_MR_MACRAW &&
                dev->sockets[sock].regs[W6300_Sn_SR] == W6300_SOCK_MACRAW) {
                w6300_socket_t *ms = &dev->sockets[sock];
                /* First RX-buffer read of a CS frame latches the VDM
                 * cursor (mirrors the PL022 path's rx_cursor_base in
                 * w6300_spi_xfer). The cursor cannot latch at CS-assert
                 * time because the frame's address phase hasn't arrived
                 * yet; latch here when the bridge serves the first byte. */
                if (!ms->rx_cursor_valid) {
                    ms->rx_cursor_base = addr;
                    ms->rx_cursor_valid = 1;
                }
            } else {
                dev->sockets[sock].rx_buf[addr % W6300_RX_BUF_SIZE] = val;
            }
        }
        break;
    }
}

/* ========================================================================
 * Initialization
 * ======================================================================== */

void w6300_init(w6300_t *dev) {
    memset(dev, 0, sizeof(*dev));

    dev->common[W6300_CIDR0] = 0x61;
    dev->common[W6300_CIDR1] = 0x00;
    dev->common[W6300_CIDR2] = 0x11;

    /* Reset defaults from the datasheet */
    dev->common[W6300_SYCR1] = 0x80;
    dev->common[W6300_PHYCR1] = 0x40;
    dev->common[W6300_RTR0]     = 0x07;
    dev->common[W6300_RTR0 + 1] = 0xD0;
    dev->common[W6300_RCR] = 0x08;
    dev->common[W6300_SLRTR0]     = 0x07;
    dev->common[W6300_SLRTR0 + 1] = 0xD0;
    dev->common[W6300_SLRCR] = 0x00;
    dev->common[W6300_SLHOPR] = 0x80;
    dev->common[W6300_SIMR] = 0xFF;  /* all sockets unmasked at reset */

    /* Default MAC: 02:00:00:00:00:01 (same convention as W5500 model) */
    dev->common[W6300_SHAR0]     = 0x02;
    dev->common[W6300_SHAR0 + 1] = 0x00;
    dev->common[W6300_SHAR0 + 2] = 0x00;
    dev->common[W6300_SHAR0 + 3] = 0x00;
    dev->common[W6300_SHAR0 + 4] = 0x00;
    dev->common[W6300_SHAR0 + 5] = 0x01;

    /* All three lock groups boot locked */
    dev->chip_locked = 1;
    dev->net_locked = 1;
    dev->phy_locked = 1;

    dev->live = 0;
    dev->vnet_port = -1;
    dev->phase = W6300_PHASE_OPCODE;
    dev->cs_active = 0;

    for (int i = 0; i < W6300_NUM_SOCKETS; i++) {
        w6300_socket_t *s = &dev->sockets[i];
        s->regs[W6300_Sn_RX_BSR] = 4;  /* 4KB */
        s->regs[W6300_Sn_TX_BSR] = 4;  /* 4KB */
        s->regs[W6300_Sn_TTLR] = 128;
        s->regs[W6300_Sn_IMR] = 0xFF;  /* datasheet reset: all socket IRQs unmasked */
        s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
        s->regs[W6300_Sn_TX_FSR0] = (W6300_TX_BUF_SIZE >> 8) & 0xFF;
        s->regs[W6300_Sn_TX_FSR0 + 1] = W6300_TX_BUF_SIZE & 0xFF;
        s->host_fd = -1;
        s->host_listen_fd = -1;
    }
}

/* ========================================================================
 * QSPI interface callbacks (single mode over the PL022 byte path)
 *
 * Frame per CS: opcode | addr_hi | addr_lo | dummy | data...
 * Dual/quad mode bits in the opcode are accepted but served byte-wise
 * (the PL022 model has no IO2/IO3 wires); guests using single mode —
 * the W6300 reset default — are bit-exact.
 * ======================================================================== */

uint8_t w6300_spi_xfer(void *ctx, uint8_t mosi) {
    w6300_t *dev = (w6300_t *)ctx;
    uint8_t miso = 0xFF;

    if (!dev->cs_active) return 0xFF;

    switch (dev->phase) {
    case W6300_PHASE_OPCODE:
        dev->opcode = mosi;
        dev->bsb = W6300_OP_BLOCK(mosi);
        dev->rw  = W6300_OP_WRITE(mosi);
        dev->phase = W6300_PHASE_ADDR_HI;
        break;

    case W6300_PHASE_ADDR_HI:
        dev->addr = (uint16_t)mosi << 8;
        dev->phase = W6300_PHASE_ADDR_LO;
        break;

    case W6300_PHASE_ADDR_LO:
        dev->addr |= mosi;
        dev->phase = W6300_PHASE_DUMMY;
        break;

    case W6300_PHASE_DUMMY:
        dev->phase = W6300_PHASE_DATA;
        break;

    case W6300_PHASE_DATA:
        if (dev->rw) {
            w6300_write_byte(dev, dev->bsb, dev->addr, mosi);
        } else {
            int type = blk_type(dev->bsb);
            int sock = blk_socket(dev->bsb);
            if (type == 3 && sock == 0 &&
                !dev->sockets[0].rx_cursor_valid) {
                dev->sockets[0].rx_cursor_base = dev->addr;
                dev->sockets[0].rx_cursor_valid = 1;
            }
            miso = w6300_read_byte(dev, dev->bsb, dev->addr);
        }
        dev->addr++;
        break;
    }

    return miso;
}

void w6300_spi_cs(void *ctx, int cs_active) {
    w6300_t *dev = (w6300_t *)ctx;
    int was = dev->cs_active;
    dev->cs_active = cs_active;

    if (cs_active && !was) {
        dev->phase = W6300_PHASE_OPCODE;
        for (int i = 0; i < W6300_NUM_SOCKETS; i++) {
            dev->sockets[i].rx_cursor_base = 0;
            dev->sockets[i].rx_cursor_valid = 0;
        }
    }
}

/* ========================================================================
 * PIO-QSPI bridge (Arduino W6300 driver path)
 *
 * The driver (wiznet_pio_qspi.c) moves every byte through a PIO state
 * machine + DMA pair instead of the PL022 SPI:
 *   write: DMA command_buf (7B: 4 opcode-nibble bytes + addr_hi +
 *          addr_lo + dummy) then payload bytes -> PIO TXF; the SM shifts
 *          them out over IO0-IO3 with SCLK side-set, CS held low via
 *          gpio_put(16) for the whole transaction.
 *   read:  DMA command_buf -> TXF, then DMA pulls rx_length bytes out of
 *          RXF; the SM clocks addr+dummy out, then shifts MISO in.
 *   framing: frame_start() = CS low, frame_end() = CS high. The driver
 *          spins on DMA TRANS_COUNT / FDEBUG TXSTALL for completion.
 *
 * The emulator's PIO engine executes the actual shift program and its
 * DMA engine does the real TXF/RXF moves, but nothing connects the
 * shifted bytes to the W6300 register file (the model sits behind the
 * PL022 byte path). This bridge snoops at the PIO register layer:
 *
 * - TX path: every 32-bit word pushed to the TXF of an armed QSPI SM is
 *   forwarded byte-wise (LSB first = DMA_SIZE_8 + bswap layout) into a
 *   per-SM stream parser with the same phases as w6300_spi_xfer
 *   (OPCODE -> ADDR_HI -> ADDR_LO -> DUMMY -> DATA). A burst of N bytes
 *   in one TXF word (DMA byte mode packs 4/word... actually one byte
 *   per word with bswap) is handled byte-by-byte. When the parser is in
 *   DATA phase, writes go straight into the model; READ frames leave
 *   the parser in DATA phase so RXF reads serve model bytes on-demand
 *   (see RX path) — stray TX DATA bytes in a READ frame are ignored.
 * - RX path: when the parser sits in the DATA phase of a READ frame,
 *   each RXF read generates one model byte with address auto-increment
 *   (mirrors the PL022 path's addr++), served instead of the (empty)
 *   hardware FIFO. Real QSPI reads clock MISO out with no further TX
 *   DATA bytes, so there is nothing to pre-queue.
 * - Arming: pico-w6300 board on + SM OUT base == 18 (the driver's
 *   `sm_config_set_out_pins(..., 18, 4)`). CS framing comes from the
 *   GPIO16 watch: CS assert resets the parser to OPCODE; bytes only
 *   forward while CS is asserted.
 * - TXSTALL/FSTAT: the existing PIO engine already advances FIFOs, so
 *   the driver's DMA-completion spins terminate naturally.
 *
 * Quad-vs-single: the driver's mk_cmd_buf() spreads each opcode bit
 * across a nibble pair (bit7..bit0 -> bytes [b7,b6],[b5,b4],[b3,b2],
 * [b1,b0] as (hi<<4)|lo) because the quad program shifts 4 bits/cycle
 * over IO0-IO3. The bridge reassembles: 4 consecutive TXF bytes form
 * one opcode byte as ((b0&1)<<7)|((b0&16)>>4<<6)... i.e. bit7 = byte0
 * bit4, bit6 = byte0 bit0, bit5 = byte1 bit4, etc. Address-hi/lo and
 * dummy travel as plain bytes (8 clocks each in quad = 2 cycles).
 * Single-mode programs (or a future single driver) send plain bytes;
 * the bridge auto-detects per frame: if the first 4 stream bytes look
 * like nibble pairs (low nibble of each is 0 or the pattern matches
 * a valid block/mode), decode quad, else single.
 *
 * LAYOUT NOTE: the bridge state struct + storage and all parser bodies
 * live AFTER the board device statics (board section below), because
 * they reference w6300_board_on / w6300_board_dev_state.
 * ======================================================================== */

/* Forward declarations (pio.c needs only these) */
int w6300_pio_is_armed(int pio_num, int sm);

/* NOTE: the remaining bridge entry points (sm_restart/pinctrl/tx_write/
 * rx_read/rx_ready/rx_level) touch parser state that needs the board
 * device, so they are defined after the board statics below. */

/* ========================================================================
 * Live Networking: Poll host sockets for incoming data
 * ======================================================================== */

/* Retry engine (see w6300_sock_retry_tick decl at file top): a SYNSENT
 * socket that never becomes writable exhausts Sn_RCR attempts of
 * Sn_RTR*100us each, then raises TIMEOUT + CLOSED like silicon. */
static void w6300_sock_retry_tick(w6300_t *dev, int sock) {
    (void)dev;
    w6300_socket_t *s = &dev->sockets[sock];
    if (s->regs[W6300_Sn_SR] != W6300_SOCK_SYNSENT) {
        s->retry_ticks = 0;
        return;
    }
    uint8_t rcr = s->regs[W6300_Sn_RCR];
    if (rcr == 0) return;  /* 0 = retry forever */
    uint16_t rtr = ((uint16_t)s->regs[W6300_Sn_RTR0] << 8) |
                   s->regs[W6300_Sn_RTR0 + 1];
    if (rtr == 0) rtr = 20000;  /* datasheet default 2000ms */
    /* ticks per attempt: RTR*100us / 1ms-per-tick, min 1. */
    uint32_t per_attempt = (uint32_t)((rtr + 9) / 10);
    if (per_attempt == 0) per_attempt = 1;
    if (++s->retry_ticks >= (uint16_t)(per_attempt * (uint32_t)rcr)) {
        s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
        s->regs[W6300_Sn_IR] |= W6300_IR_TIMEOUT;
        if (s->host_fd >= 0) { close(s->host_fd); s->host_fd = -1; }
        s->retry_ticks = 0;
        w6300_board_refresh_int();
    }
}

void w6300_poll(w6300_t *dev) {
    if (!dev->live) return;

    /* TCNTR: free-running 1ms ticker (silicon: 1ms per LSB). poll() is
     * the emulator's 1ms-ish tick source (main-loop + WASM step pump). */
    {
        uint16_t t = ((uint16_t)dev->common[W6300_TCNTR0] << 8) |
                     dev->common[W6300_TCNTR0 + 1];
        t++;
        dev->common[W6300_TCNTR0] = (t >> 8) & 0xFF;
        dev->common[W6300_TCNTR0 + 1] = t & 0xFF;
    }

    for (int i = 0; i < W6300_NUM_SOCKETS; i++) {
        w6300_socket_t *s = &dev->sockets[i];

        if (s->host_listen_fd >= 0 &&
            s->regs[W6300_Sn_SR] == W6300_SOCK_LISTEN) {
            struct pollfd pfd = { .fd = s->host_listen_fd, .events = POLLIN };
            if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
                /* sockaddr_storage: a v6 listener's client is v6; the
                 * old sockaddr_in truncated it (DIPR garbage, port ok
                 * by luck). Mirror v6 peers into DIP6R/DPORTR. */
                struct sockaddr_storage client;
                socklen_t clen = sizeof(client);
                int cfd = accept(s->host_listen_fd, (struct sockaddr *)&client, &clen);
                if (cfd >= 0) {
                    set_sock_nonblock(cfd);
                    if (s->host_fd >= 0) close(s->host_fd);
                    s->host_fd = cfd;
                    s->regs[W6300_Sn_SR] = W6300_SOCK_ESTABLISHED;
                    uint16_t port;
                    if (client.ss_family == AF_INET6) {
                        struct sockaddr_in6 *c6 = (struct sockaddr_in6 *)&client;
                        for (int k = 0; k < 16; k++)
                            s->regs[W6300_Sn_DIP6R0 + k] = c6->sin6_addr.s6_addr[k];
                        port = ntohs(c6->sin6_port);
                    } else {
                        struct sockaddr_in *c4 = (struct sockaddr_in *)&client;
                        uint32_t ip = ntohl(c4->sin_addr.s_addr);
                        s->regs[W6300_Sn_DIPR0]     = (ip >> 24) & 0xFF;
                        s->regs[W6300_Sn_DIPR0 + 1] = (ip >> 16) & 0xFF;
                        s->regs[W6300_Sn_DIPR0 + 2] = (ip >>  8) & 0xFF;
                        s->regs[W6300_Sn_DIPR0 + 3] = ip & 0xFF;
                        port = ntohs(c4->sin_port);
                    }
                    s->regs[W6300_Sn_DPORTR0]     = (port >> 8) & 0xFF;
                    s->regs[W6300_Sn_DPORTR0 + 1] = port & 0xFF;
                    s->regs[W6300_Sn_IR] |= W6300_IR_CON;
                }
            }
        }

        if (s->regs[W6300_Sn_SR] == W6300_SOCK_SYNSENT) {
            if (s->host_fd >= 0) {
            struct pollfd pfd = { .fd = s->host_fd, .events = POLLOUT };
            if (poll(&pfd, 1, 0) > 0 &&
                (pfd.revents & (POLLOUT | POLLERR | POLLHUP))) {
                int err = 0;
                socklen_t elen = sizeof(err);
                getsockopt(s->host_fd, SOL_SOCKET, SO_ERROR, &err, &elen);
                if (err == 0) {
                    s->regs[W6300_Sn_SR] = W6300_SOCK_ESTABLISHED;
                    s->regs[W6300_Sn_IR] |= W6300_IR_CON;
                    s->retry_ticks = 0;
                } else {
                    s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
                    close(s->host_fd);
                    s->host_fd = -1;
                    s->retry_ticks = 0;
                }
            } else {
                w6300_sock_retry_tick(dev, i);
            }
            } else {
                /* Offline SYNSENT (no host fd, e.g. WASM proxy wait or
                 * stub): still run the retry budget so a guest that
                 * never gets CON sees TIMEOUT like silicon. */
                w6300_sock_retry_tick(dev, i);
            }
        }

        /* IPRAW sockets also receive (raw datagrams have no header
         * framing in the model: payload lands as-is; ioLibrary parity
         * would need IP-header parsing, which no in-tree guest uses). */
        if (s->host_fd >= 0 &&
            (s->regs[W6300_Sn_SR] == W6300_SOCK_ESTABLISHED ||
             s->regs[W6300_Sn_SR] == W6300_SOCK_UDP ||
             s->regs[W6300_Sn_SR] == W6300_SOCK_IPRAW4 ||
             s->regs[W6300_Sn_SR] == W6300_SOCK_IPRAW6)) {

            uint16_t rx_rsr = ((uint16_t)s->regs[W6300_Sn_RX_RSR0] << 8) |
                              s->regs[W6300_Sn_RX_RSR0 + 1];
            uint16_t free_space = W6300_RX_BUF_SIZE - rx_rsr;
            if (free_space == 0) continue;

            /* IPv6 sockets speak recvfrom/recv on an AF_INET6 fd; the
             * UDP header layout is identical (WIZnet packs the v4-mapped
             * 4B source for v4 and the raw 16B for v6 — the ioLibrary
             * UDP6 path consumes DIP6R-indexed reads, so store the peer
             * bytes the same way: 8B v4 header, 24B v6 header). */
            int sock_is_v6 = 0;
            {
                struct sockaddr_storage ss;
                socklen_t sl = sizeof(ss);
                if (getsockname(s->host_fd, (struct sockaddr *)&ss,
                                &sl) == 0 && ss.ss_family == AF_INET6)
                    sock_is_v6 = 1;
            }
            struct pollfd pfd = { .fd = s->host_fd, .events = POLLIN };
            if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
                uint16_t rx_wr = ((uint16_t)s->regs[W6300_Sn_RX_WR0] << 8) |
                                 s->regs[W6300_Sn_RX_WR0 + 1];

                uint8_t tmp[W6300_RX_BUF_SIZE];
                int is_udp = (s->regs[W6300_Sn_SR] == W6300_SOCK_UDP);
                uint8_t hdr[24];
                int hl = 0;
                ssize_t n = -1;
                if (is_udp) {
                    if (sock_is_v6) {
                        if (free_space > 24) {
                            struct sockaddr_in6 src6;
                            socklen_t slen = sizeof(src6);
                            size_t room = (size_t)free_space - 24;
                            if (room > sizeof(tmp)) room = sizeof(tmp);
                            n = recvfrom(s->host_fd, tmp, room,
                                         0, (struct sockaddr *)&src6, &slen);
                            if (n > 0) {
                                for (int i = 0; i < 16; i++)
                                    hdr[i] = src6.sin6_addr.s6_addr[i];
                                uint16_t sport = ntohs(src6.sin6_port);
                                hdr[16] = (sport >> 8) & 0xFF; hdr[17] = sport & 0xFF;
                                hdr[18] = 0; hdr[19] = 0;
                                hdr[20] = ((uint16_t)n >> 8) & 0xFF; hdr[21] = (uint16_t)n & 0xFF;
                                hl = 22;
                                /* Mirror the peer into DIP6R/DPORTR so a
                                 * SEND6 reply goes back where it came. */
                                for (int i = 0; i < 16; i++)
                                    s->regs[W6300_Sn_DIP6R0 + i] = hdr[i];
                                s->regs[W6300_Sn_DPORTR0] = hdr[16];
                                s->regs[W6300_Sn_DPORTR0 + 1] = hdr[17];
                            }
                        }
                    } else if (free_space > 8) {
                        struct sockaddr_in src;
                        socklen_t slen = sizeof(src);
                        size_t room = (size_t)free_space - 8;
                        if (room > sizeof(tmp)) room = sizeof(tmp);
                        n = recvfrom(s->host_fd, tmp, room,
                                     0, (struct sockaddr *)&src, &slen);
                        if (n > 0) {
                            uint32_t sip = ntohl(src.sin_addr.s_addr);
                            uint16_t sport = ntohs(src.sin_port);
                            hdr[0] = (sip >> 24) & 0xFF; hdr[1] = (sip >> 16) & 0xFF;
                            hdr[2] = (sip >> 8) & 0xFF;  hdr[3] = sip & 0xFF;
                            hdr[4] = (sport >> 8) & 0xFF; hdr[5] = sport & 0xFF;
                            hdr[6] = ((uint16_t)n >> 8) & 0xFF; hdr[7] = (uint16_t)n & 0xFF;
                            hl = 8;
                        }
                    }
                } else {
                    size_t room = free_space < sizeof(tmp) ? free_space : sizeof(tmp);
                    n = recv(s->host_fd, tmp, room, 0);
                }
                if (n > 0) {
                    for (int j = 0; j < hl; j++) {
                        s->rx_buf[(rx_wr + (uint16_t)j) % W6300_RX_BUF_SIZE] = hdr[j];
                    }
                    for (ssize_t j = 0; j < n; j++) {
                        s->rx_buf[(rx_wr + (uint16_t)(hl + j)) % W6300_RX_BUF_SIZE] = tmp[j];
                    }
                    uint16_t total = (uint16_t)(hl + n);
                    rx_wr = (uint16_t)(rx_wr + total);
                    s->regs[W6300_Sn_RX_WR0]     = (rx_wr >> 8) & 0xFF;
                    s->regs[W6300_Sn_RX_WR0 + 1] = rx_wr & 0xFF;
                    rx_rsr = (uint16_t)(rx_rsr + total);
                    s->regs[W6300_Sn_RX_RSR0]     = (rx_rsr >> 8) & 0xFF;
                    s->regs[W6300_Sn_RX_RSR0 + 1] = rx_rsr & 0xFF;
                    s->regs[W6300_Sn_IR] |= W6300_IR_RECV;
                } else if (n == 0) {
                    s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
                    s->regs[W6300_Sn_IR] |= W6300_IR_DISCON;
                    close(s->host_fd);
                    s->host_fd = -1;
                }
            }

            if (s->host_fd >= 0) {
                struct pollfd pfd2 = { .fd = s->host_fd, .events = 0 };
                if (poll(&pfd2, 1, 0) > 0 &&
                    (pfd2.revents & (POLLERR | POLLHUP))) {
                    s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
                    s->regs[W6300_Sn_IR] |= W6300_IR_DISCON;
                    close(s->host_fd);
                    s->host_fd = -1;
                }
            }
        }
    }
}

void w6300_set_live(w6300_t *dev, int enable) {
    dev->live = enable;
    if (enable) {
        fprintf(stderr, "[W6300] Live networking enabled\n");
    }
}

/* ========================================================================
 * W6300-EVB-Pico board variant (RP2040; -board pico-w6300)
 * W6300-EVB-Pico2 board variant (RP2350; -board pico-w6300-2)
 *
 * Real wiring (WIZnet docs): INTn=GPIO15 CSn=GPIO16 SCLK=GPIO17
 * IO0=GPIO18 IO1=GPIO19 IO2=GPIO20 IO3=GPIO21 RSTn=GPIO22.
 * Single-SPI mode rides the normal PL022 path; the board watches
 * CSn/RSTn edges and drives INTn active-low.
 * ======================================================================== */

#include "spi.h"
#include "gpio.h"
#include "pio.h"

static w6300_t w6300_board_dev_state;
static int w6300_board_on = 0;
static int w6300_board_spi_num = W6300_BOARD_SPI_DEFAULT;

/* PIO-QSPI bridge state lives here (after the board device) so the
 * parser helpers above can reference the board state. */
#define W6300_QSPI_MAX_SM 4

typedef struct {
    int armed;              /* OUT base==18 seen on this SM */
    int phase;              /* mirror of w6300_phase_t (0..4) */
    uint8_t bsb;
    int rw;
    uint16_t addr;
    /* quad detection: first 4 bytes of a frame buffered here */
    uint8_t hdr[8];
    int hdr_len;
    int quad;               /* -1 unknown, 0 single, 1 quad */
    /* No RX response queue: reads are served on-demand in
     * w6300_pio_rx_read (one model byte per RXF read, addr++).
     * Real QSPI reads send opcode+addr+dummy via TX then clock MISO
     * bytes out of RXF with no further TX DATA bytes, so there is
     * nothing to queue — the old queue never filled and only the
     * on-demand path ever served. */
    /* Pre-DMA setup words: the driver does 2x pio_sm_put (X/Y bit
     * counts) before every DMA burst. Those TXF pushes are NOT QSPI
     * stream bytes and must be skipped (CYW43 precedent:
     * pio_pre_dma_skip in cyw43.c). Set on SM restart, consumed on
     * TXF push. */
    int setup_skip;
} w6300_qspi_sm_t;

static w6300_qspi_sm_t w6300_qspi[3][W6300_QSPI_MAX_SM];

/* Board on-flag lives below with the device; the bridge checks it via
 * w6300_board_enabled() (declared in w6300.h) to avoid a use-before-def. */
/* (W6300_TR is defined once at file top.) */

/* --- bridge parser bodies (need the board statics above) --- */

static void w6300_qspi_push_byte(int pio_num, int sm, uint8_t b);

static int w6300_qspi_resp_pending(int pio_num, int sm) {
    /* On-demand RX: a queued byte exists iff the parser sits in the
     * DATA phase of a READ frame (w6300_pio_rx_read generates one
     * model byte per RXF read). Never any stored depth. */
    w6300_qspi_sm_t *q = &w6300_qspi[pio_num][sm];
    return (q->phase >= 4 && !q->rw) ? 1 : 0;
}

static int w6300_qspi_resp_level(int pio_num, int sm) {
    return w6300_qspi_resp_pending(pio_num, sm) ? 1 : 0;
}

/* Reassemble one opcode byte from 4 quad-nibble stream bytes.
 * mk_cmd_buf: pdst[i] = ((opcode>>(7-2i))&1)<<4 | ((opcode>>(6-2i))&1).
 * So bit(7-2i) = byte[i] bit4, bit(6-2i) = byte[i] bit0. */
static uint8_t w6300_qspi_quad_opcode(const uint8_t *nb) {
    uint8_t op = 0;
    for (int i = 0; i < 4; i++) {
        if (nb[i] & 0x10) op |= (uint8_t)(1u << (7 - 2 * i));
        if (nb[i] & 0x01) op |= (uint8_t)(1u << (6 - 2 * i));
    }
    return op;
}

/* Feed one stream byte (already quad-reassembled where applicable)
 * through the frame parser into the board model. Writes go straight
 * into the model; READ frames just park the parser in DATA phase so
 * RXF reads serve model bytes on-demand (see w6300_pio_rx_read).
 *
 * CS handling: the driver holds CS low (gpio_put 16) for the whole
 * transaction, and the GPIO16 watch mirrors that into dev->cs_active.
 * If a frame arrives while cs_active is somehow clear (CS edge raced
 * the first TXF push), treat the OPCODE byte as an implicit frame
 * start rather than dropping the transaction. */
static void w6300_qspi_frame_byte(int pio_num, int sm, uint8_t b) {
    w6300_t *dev = &w6300_board_dev_state;
    w6300_qspi_sm_t *q = &w6300_qspi[pio_num][sm];

    W6300_TR("FRAME pio%d sm%d b=0x%02X phase=%d cs=%d", pio_num, sm, b,
             q->phase, dev->cs_active);
    if (!dev->cs_active && q->phase != 0) return;

    switch (q->phase) {
    case 0: /* OPCODE */
        q->bsb = W6300_OP_BLOCK(b);
        q->rw = W6300_OP_WRITE(b);
        q->phase = 1;
        break;
    case 1: /* ADDR_HI */
        q->addr = (uint16_t)b << 8;
        q->phase = 2;
        break;
    case 2: /* ADDR_LO */
        q->addr |= b;
        q->phase = 3;
        break;
    case 3: /* DUMMY */
        q->phase = 4;
        break;
    default: /* DATA */
        if (q->rw) {
            w6300_write_byte(dev, q->bsb, q->addr, b);
            q->addr++;
        } else {
            /* READ frames generate model bytes on RXF demand (see
             * w6300_pio_rx_read); stray TX DATA bytes (e.g. the
             * dummy-phase tail the DMA replays) must not queue
             * duplicates or advance the address. */
            W6300_TR("READ-DATA-TX-IGN pio%d sm%d b=0x%02X bsb=%d addr=0x%04X",
                     pio_num, sm, b, q->bsb, q->addr);
        }
        break;
    }
}

static void w6300_qspi_push_byte(int pio_num, int sm, uint8_t b);

/* (w6300_pio_cs_assert is defined once at file end, after the bridge
 * state storage.) */

/* INTn pin state: active-low, masked by the silicon chain
 *   Sn_IR[n] & Sn_IMR[n] -> SIR[n] & SIMR[n] -> INTn,
 * plus common IR & IMR -> INTn — all ANDed with global IEN
 * (SYCR1 bit7). w6300_board_refresh_int() runs on every register
 * path that can change an IR bit (shared by the PL022 path and the
 * PIO-QSPI bridge, which both funnel through w6300_read/write_byte);
 * w6300_board_update_int() is the poll-loop entry with the same
 * computation. */
static int w6300_int_pending(w6300_t *dev) {
    if ((dev->common[W6300_SYCR1] & W6300_SYCR1_IEN) == 0) return 0;
    if (dev->common[W6300_IR] & dev->common[W6300_IMR]) return 1;
    if (dev->common[W6300_SLIR] & dev->common[W6300_SLIMR]) return 1;
    for (int i = 0; i < W6300_NUM_SOCKETS; i++) {
        uint8_t ir = dev->sockets[i].regs[W6300_Sn_IR];
        uint8_t imr = dev->sockets[i].regs[W6300_Sn_IMR];
        if ((ir & imr) == 0) continue;
        if ((dev->common[W6300_SIMR] & (1u << i)) == 0) continue;
        return 1;
    }
    return 0;
}

static void w6300_board_refresh_int(void) {
    if (!w6300_board_on) return;
    int pending = w6300_int_pending(&w6300_board_dev_state);
    gpio_set_direction(W6300_BOARD_INT_PIN, 0);
    gpio_mark_driven(W6300_BOARD_INT_PIN);
    gpio_set_input_pin(W6300_BOARD_INT_PIN, pending ? 0 : 1);
}

int w6300_board_enabled(void) { return w6300_board_on; }
int w6300_board_spi(void) { return w6300_board_spi_num; }
w6300_t *w6300_board_dev(void) { return &w6300_board_dev_state; }

void w6300_board_update_int(void) {
    if (!w6300_board_on) return;
    int pending = w6300_int_pending(&w6300_board_dev_state);
    gpio_set_direction(W6300_BOARD_INT_PIN, 0);
    gpio_mark_driven(W6300_BOARD_INT_PIN);
    gpio_set_input_pin(W6300_BOARD_INT_PIN, pending ? 0 : 1);
}

static int w6300_board_prev_cs = 1;
static int w6300_board_prev_rst = 1;

void w6300_board_gpio_write(uint32_t pin, uint32_t value) {
    int v = value ? 1 : 0;
    if (pin == W6300_BOARD_CS_PIN) {
        if (v == w6300_board_prev_cs) return;
        w6300_board_prev_cs = v;
        w6300_spi_cs(&w6300_board_dev_state, v ? 0 : 1);
        if (v) {
            /* CS high (frame_end): the transaction is over. Nothing to
             * discard: reads are served on-demand straight from the
             * model (no response queue), and the parser re-syncs on
             * the next restart/CS edge. */
        }
    } else if (pin == W6300_BOARD_RST_PIN) {
        if (v == w6300_board_prev_rst) return;
        w6300_board_prev_rst = v;
        if (!v) {
            memset(&w6300_board_dev_state.common, 0,
                   sizeof(w6300_board_dev_state.common));
            for (int i = 0; i < W6300_NUM_SOCKETS; i++)
                memset(w6300_board_dev_state.sockets[i].regs, 0,
                       sizeof(w6300_board_dev_state.sockets[i].regs));
            w6300_board_dev_state.phase = W6300_PHASE_OPCODE;
        } else {
            int live = w6300_board_dev_state.live;
            w6300_init(&w6300_board_dev_state);
            w6300_board_dev_state.live = live;
            w6300_board_update_int();
        }
    }
}

void w6300_board_poll(void) {
    if (!w6300_board_on) return;
    if (w6300_board_dev_state.live)
        w6300_poll(&w6300_board_dev_state);
    w6300_board_update_int();
}

void w6300_board_attach(int spi_num, int live) {
    if (spi_num < 0 || spi_num > 1) spi_num = W6300_BOARD_SPI_DEFAULT;
    w6300_init(&w6300_board_dev_state);
    w6300_board_dev_state.live = live ? 1 : 0;
    w6300_board_spi_num = spi_num;
    spi_attach_device(spi_num, w6300_spi_xfer, w6300_spi_cs,
                      &w6300_board_dev_state);
    w6300_board_on = 1;
    gpio_set_direction(W6300_BOARD_INT_PIN, 0);
    gpio_mark_driven(W6300_BOARD_INT_PIN);
    gpio_set_input_pin(W6300_BOARD_INT_PIN, 1);
    gpio_write32(SIO_BASE_GPIO + 0x24,
                 (1u << W6300_BOARD_CS_PIN) | (1u << W6300_BOARD_RST_PIN));
    gpio_write32(SIO_BASE_GPIO + 0x14,
                 (1u << W6300_BOARD_CS_PIN) | (1u << W6300_BOARD_RST_PIN));
    w6300_board_prev_cs = 1;
    w6300_board_prev_rst = 1;
    w6300_spi_cs(&w6300_board_dev_state, 0);
    w6300_board_update_int();
    fprintf(stderr, "[W6300] pico-w6300 board on SPI%d (CS16/RST22/INT15)%s\n",
            spi_num, live ? " live" : " (stub)");
}

void w6300_board_detach(void) {
    w6300_board_on = 0;
    /* Detach always fully resets bridge + model state, even if called
     * when already off (unit tests call detach-then-attach; reset_cpu
     * clears GPIO so stale CS edges must not leak a half-frame). */
    for (int b = 0; b < 3; b++)
        for (int sm = 0; sm < W6300_QSPI_MAX_SM; sm++) {
            w6300_qspi[b][sm].phase = 0;
            w6300_qspi[b][sm].hdr_len = 0;
            w6300_qspi[b][sm].quad = -1;
            w6300_qspi[b][sm].armed = 0;
        }
    gpio_unmark_driven(W6300_BOARD_INT_PIN);
    w6300_board_dev_state.live = 0;
    for (int i = 0; i < W6300_NUM_SOCKETS; i++) {
        w6300_socket_t *s = &w6300_board_dev_state.sockets[i];
        if (s->host_fd >= 0) { close(s->host_fd); s->host_fd = -1; }
        if (s->host_listen_fd >= 0) { close(s->host_listen_fd); s->host_listen_fd = -1; }
    }
    if (w6300_board_spi_num >= 0 && w6300_board_spi_num <= 1 &&
        spi_state[w6300_board_spi_num].device.ctx == &w6300_board_dev_state &&
        spi_state[w6300_board_spi_num].device.xfer == w6300_spi_xfer) {
        spi_state[w6300_board_spi_num].device.xfer = NULL;
        spi_state[w6300_board_spi_num].device.cs = NULL;
        spi_state[w6300_board_spi_num].device.ctx = NULL;
    }
}

void w6300_board_reattach(void) {
    if (!w6300_board_on) return;
    spi_attach_device(w6300_board_spi_num, w6300_spi_xfer, w6300_spi_cs,
                      &w6300_board_dev_state);
}

void w6300_board_set_live(int live) {
    if (!w6300_board_on) return;
    w6300_board_dev_state.live = live ? 1 : 0;
}

#ifdef __EMSCRIPTEN__
int picoemu_w6300_dev_push_rx(w6300_t *dev, int sock, const uint8_t *data, int len) {
    if (!dev || sock < 0 || sock >= W6300_NUM_SOCKETS || !data || len <= 0) return -1;
    w6300_socket_t *s = &dev->sockets[sock];
    uint16_t rx_rsr = ((uint16_t)s->regs[W6300_Sn_RX_RSR0] << 8) |
                      s->regs[W6300_Sn_RX_RSR0 + 1];
    uint16_t free_space = W6300_RX_BUF_SIZE - rx_rsr;
    if (free_space == 0) return 0;
    if (len > free_space) len = free_space;
    uint16_t rx_wr = ((uint16_t)s->regs[W6300_Sn_RX_WR0] << 8) |
                     s->regs[W6300_Sn_RX_WR0 + 1];
    for (int i = 0; i < len; i++)
        s->rx_buf[(rx_wr + (uint16_t)i) % W6300_RX_BUF_SIZE] = data[i];
    rx_wr = (uint16_t)(rx_wr + (uint16_t)len);
    s->regs[W6300_Sn_RX_WR0]     = (rx_wr >> 8) & 0xFF;
    s->regs[W6300_Sn_RX_WR0 + 1] = rx_wr & 0xFF;
    rx_rsr = (uint16_t)(rx_rsr + (uint16_t)len);
    s->regs[W6300_Sn_RX_RSR0]     = (rx_rsr >> 8) & 0xFF;
    s->regs[W6300_Sn_RX_RSR0 + 1] = rx_rsr & 0xFF;
    s->regs[W6300_Sn_IR] |= W6300_IR_RECV;
    return len;
}
int picoemu_w6300_dev_push_status(w6300_t *dev, int sock, int code) {
    if (!dev || sock < 0 || sock >= W6300_NUM_SOCKETS) return -1;
    w6300_socket_t *s = &dev->sockets[sock];
    if (code) {
        if (s->regs[W6300_Sn_SR] == W6300_SOCK_INIT ||
            s->regs[W6300_Sn_SR] == W6300_SOCK_LISTEN ||
            s->regs[W6300_Sn_SR] == W6300_SOCK_CLOSED) {
            s->regs[W6300_Sn_SR] = W6300_SOCK_ESTABLISHED;
        }
        s->regs[W6300_Sn_IR] |= W6300_IR_CON;
    } else {
        s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
        s->regs[W6300_Sn_IR] |= W6300_IR_DISCON;
    }
    return 0;
}
#endif

/* ========================================================================
 * PIO-QSPI bridge: pio.c entry points + parser bodies (need the board
 * statics above, so they live here at file end).
 * Trace macro is defined once near the top (before the parser bodies).
 * ======================================================================== */

void w6300_pio_cs_assert(void) {
    for (int b = 0; b < 3; b++)
        for (int sm = 0; sm < W6300_QSPI_MAX_SM; sm++) {
            w6300_qspi[b][sm].phase = 0;
            w6300_qspi[b][sm].hdr_len = 0;
            w6300_qspi[b][sm].quad = -1;
        }
}

static void w6300_qspi_push_byte(int pio_num, int sm, uint8_t b) {
    w6300_qspi_sm_t *q = &w6300_qspi[pio_num][sm];

    /* Header accumulation for quad/single auto-detect: the first 4
     * stream bytes are either 4 nibble-pair bytes (quad) or
     * opcode+addr_hi+addr_lo+dummy (single). mk_cmd_buf emits only
     * 0x00/0x01/0x10/0x11 per byte, BUT a single-mode frame can also
     * match (e.g. opcode 0x00 + zero addrs). Disambiguate with the
     * QSPI mode bits: the quad driver always sets QSPI_QUAD_MODE
     * (0x80) in the opcode, so a valid quad header reassembles to an
     * opcode with bit7 set. Nibble-set match AND (reassembled & 0x80)
     * -> quad; otherwise replay the 4 bytes as single. */
    if (q->quad < 0 && q->hdr_len < 4) {
        q->hdr[q->hdr_len++] = b;
        if (q->hdr_len == 4) {
            int i;
            for (i = 0; i < 4; i++) {
                uint8_t v = q->hdr[i];
                if (v != 0x00 && v != 0x01 && v != 0x10 && v != 0x11) break;
            }
            uint8_t op = w6300_qspi_quad_opcode(q->hdr);
            if (i == 4 && (op & 0x80)) {
                q->quad = 1;
                w6300_qspi_frame_byte(pio_num, sm, op);
                q->hdr_len = 0;
            } else {
                q->quad = 0;
                for (i = 0; i < 4; i++)
                    w6300_qspi_frame_byte(pio_num, sm, q->hdr[i]);
                q->hdr_len = 0;
            }
        }
        return;
    }
    if (q->quad == 1 && q->phase == 0 && q->hdr_len < 4) {
        /* Quad OPCODE only: the 4 opcode nibble bytes. Everything after
         * (addr_hi/lo + dummy + payload) crosses the TXF stream as
         * single bytes — the driver's second DMA programs the raw tx
         * buffer straight at TXF, and the PIO program's OUT shift
         * serializes whatever width the wrap segment implies. Only the
         * mk_cmd_buf opcode needs nibble reassembly. */
        q->hdr[q->hdr_len++] = b;
        if (q->hdr_len == 4) {
            w6300_qspi_frame_byte(pio_num, sm, w6300_qspi_quad_opcode(q->hdr));
            q->hdr_len = 0;
        }
        return;
    }
    w6300_qspi_frame_byte(pio_num, sm, b);
}

void w6300_pio_sm_restart(int pio_num, int sm) {
    if (pio_num < 0 || pio_num > 2 || sm < 0 || sm > 3) return;
    w6300_qspi_sm_t *q = &w6300_qspi[pio_num][sm];
    q->phase = 0;
    q->hdr_len = 0;
    q->quad = -1;
    /* NOTE: reads are served on-demand from the model (no response
     * queue), so there is nothing to preserve or clear here — just
     * re-sync the parser. The driver's read path calls pio_sm_restart()
     * at transaction start, then programs the TXF stream; the RX DMA
     * pulls model bytes AFTER the TX DMA completes, all within one CS
     * frame. */
    /* The driver follows every restart with 2x pio_sm_put bit-count
     * setup words (X then Y) before the DMA burst. Skip exactly those
     * two TXF pushes so they never enter the frame parser. */
    q->setup_skip = 2;
    /* NOTE: do NOT consult w6300_pio_is_armed here (it reports the armed
     * flag, which would make arming impossible). Restart only resets the
     * parser; PINCTRL writes do the arming. */
    (void)q;
}

void w6300_pio_pinctrl(int pio_num, int sm) {
    if (pio_num < 0 || pio_num > 2 || sm < 0 || sm > 3) return;
    if (!w6300_board_on) return;
    /* OUT base == 18 (IO0): the wiznet_pio_qspi program's out-pins
     * config, applied via pio_sm_set_config -> PINCTRL write. Read the
     * just-written PINCTRL directly (the armed flag is what
     * w6300_pio_is_armed reports — don't consult it here or arming can
     * never engage). */
    extern pio_block_t pio_state[];
    uint32_t pinctrl = pio_state[pio_num].sm[sm].pinctrl;
    if (((pinctrl >> 0) & 0x1F) == 18) {
        w6300_qspi[pio_num][sm].armed = 1;
        W6300_TR("ARM pio%d sm%d (OUT base 18)", pio_num, sm);
    } else {
        w6300_qspi[pio_num][sm].armed = 0;
    }
}

int w6300_pio_is_armed(int pio_num, int sm) {
    if (pio_num < 0 || pio_num > 2 || sm < 0 || sm > 3) return 0;
    if (!w6300_board_on) return 0;
    return w6300_qspi[pio_num][sm].armed;
}

int w6300_pio_rx_level(int pio_num, int sm) {
    if (pio_num < 0 || pio_num > 2 || sm < 0 || sm > 3) return -1;
    if (!w6300_board_on || !w6300_qspi[pio_num][sm].armed) return -1;
    return w6300_qspi_resp_level(pio_num, sm);
}

void w6300_pio_tx_write(int pio_num, int sm, uint32_t val) {
    if (!w6300_board_on) return;
    if (pio_num < 0 || pio_num > 2 || sm < 0 || sm > 3) return;
    /* NOTE: no OUT-base re-gate here — arming is decided by PINCTRL
     * writes (w6300_pio_pinctrl). TXF pushes forward whenever armed. */
    if (!w6300_qspi[pio_num][sm].armed) return;
    W6300_TR("TXF pio%d sm%d val=0x%08X skip=%d phase=%d hdr=%d quad=%d cs=%d eff16=%d",
             pio_num, sm, val, w6300_qspi[pio_num][sm].setup_skip,
             w6300_qspi[pio_num][sm].phase, w6300_qspi[pio_num][sm].hdr_len,
             w6300_qspi[pio_num][sm].quad, w6300_board_dev_state.cs_active,
             (gpio_effective_pins() >> 16) & 1u);
    /* Skip the pre-DMA bit-count setup words (pio_sm_put X/Y before
     * each DMA burst) — they are SM configuration, not QSPI bytes. */
    if (w6300_qspi[pio_num][sm].setup_skip > 0) {
        w6300_qspi[pio_num][sm].setup_skip--;
        return;
    }
    w6300_qspi[pio_num][sm].armed = 1;
    /* DMA_SIZE_8 + bswap: each TXF word carries one payload byte in
     * the LSB. Forward LSB per word in write order == stream order.
     * Frame sync: the driver always brackets a transaction with
     * frame_start (CS low) / frame_end (CS high) via gpio_put(16), and
     * the GPIO16 watch drives dev->cs_active — but CS edges can race
     * the first TXF push through different register paths, so ALSO
     * treat the start of a fresh parser frame (phase 0, no header
     * bytes yet) as an implicit frame start. This keeps single-byte
     * probe reads (VERSIONR/CIDR polls) aligned even if their CS edge
     * was consumed before the bridge armed.
     * CS SOURCE OF TRUTH: the driver's frame_start/frame_end path is
     * gpio_put(16) (SIO OUT_SET/OUT_CLR -> OE-gated effective level).
     * The GPIO16 watch (w6300_board_gpio_write) can lag the first TXF
     * push when frame_start's gpio_set_function(PIO) + OUT_CLR land in
     * the same window, so re-derive CS from the effective pin level
     * here: eff16==0 means the guest is driving CS low right now. */
    if (!w6300_board_dev_state.cs_active) {
        w6300_qspi_sm_t *qq = &w6300_qspi[pio_num][sm];
        if (qq->phase == 0 && qq->hdr_len == 0) {
            if (((gpio_effective_pins() >> 16) & 1u) == 0)
                w6300_board_dev_state.cs_active = 1;
        }
    }
    w6300_qspi_push_byte(pio_num, sm, (uint8_t)(val & 0xFF));
}

uint32_t w6300_pio_rx_read(int pio_num, int sm) {
    if (pio_num < 0 || pio_num > 2 || sm < 0 || sm > 3) return 0;
    if (!w6300_board_on || !w6300_qspi[pio_num][sm].armed) return 0;
    /* On-demand read: real QSPI reads send opcode+addr+dummy via TX,
     * then clock MISO bytes out of RXF with NO further TX DATA bytes.
     * When the parser sits in DATA phase of a READ frame, generate one
     * model byte per RXF read with address auto-increment (mirrors the
     * PL022 path's addr++ in w6300_spi_xfer). */
    {
        w6300_qspi_sm_t *q = &w6300_qspi[pio_num][sm];
        if (q->phase >= 4 && !q->rw) {
            w6300_t *dev = &w6300_board_dev_state;
            uint8_t r = w6300_read_byte(dev, q->bsb, q->addr);
            W6300_TR("RXF pio%d sm%d on-demand bsb=%d addr=0x%04X -> 0x%02X",
                     pio_num, sm, q->bsb, q->addr, r);
            q->addr++;
            return r;
        }
    }
    W6300_TR("RXF pio%d sm%d EMPTY (armed=%d)", pio_num, sm,
             (pio_num >= 0 && sm >= 0) ? w6300_qspi[pio_num][sm].armed : -1);
    return 0;
}

int w6300_pio_rx_ready(int pio_num, int sm) {
    if (pio_num < 0 || pio_num > 2 || sm < 0 || sm > 3) return 0;
    if (!w6300_board_on || !w6300_qspi[pio_num][sm].armed) return 0;
    /* Data available iff the parser sits in the DATA phase of a READ
     * frame (on-demand generation serves it; see w6300_pio_rx_read).
     * Report ready so FSTAT clears RXEMPTY for the armed SM. */
    return w6300_qspi_resp_pending(pio_num, sm);
}

/* SM-EXEC OUT X/Y consume hook (called from pio.c forced-exec): drop one
 * pending TXF word from the bridge parser's view. Returns 1 if a word
 * was consumed. */
int w6300_pio_exec_out_drop(int pio_num, int sm) {
    if (pio_num < 0 || pio_num > 2 || sm < 0 || sm > 3) return 0;
    w6300_qspi_sm_t *q = &w6300_qspi[pio_num][sm];
    if (q->setup_skip > 0) {
        q->setup_skip--;
        return 1;
    }
    return 0;
}
