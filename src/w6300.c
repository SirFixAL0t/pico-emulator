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
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "w6300.h"
#include "vnet.h"

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
    uint16_t base = s->tx_dirty_valid ? s->tx_dirty_base : 0;
    uint16_t data_len = s->tx_dirty_valid ? s->tx_dirty_len : 0;
    if (!s->tx_dirty_valid) {
        uint16_t tx_rd = ((uint16_t)s->regs[W6300_Sn_TX_RD0] << 8) |
                         s->regs[W6300_Sn_TX_RD0 + 1];
        base = tx_rd;
        data_len = (uint16_t)(tx_wr - tx_rd);
    }
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

static void w6300_process_socket_cmd(w6300_t *dev, int sock) {
    w6300_socket_t *s = &dev->sockets[sock];
    uint8_t cmd = s->regs[W6300_Sn_CR];
    uint8_t mode = s->regs[W6300_Sn_MR];

    if (cmd == 0) return;

    switch (cmd) {
    case W6300_CMD_OPEN:
        if (w6300_mode_is_tcp(mode)) {
            s->regs[W6300_Sn_SR] = W6300_SOCK_INIT;
            if (dev->live) {
                w6300_close_host_sock(s);
                s->host_fd = socket(AF_INET, SOCK_STREAM, 0);
                if (s->host_fd >= 0) set_sock_nonblock(s->host_fd);
            }
        } else if (w6300_mode_is_udp(mode)) {
            s->regs[W6300_Sn_SR] = W6300_SOCK_UDP;
            if (dev->live) {
                w6300_close_host_sock(s);
                s->host_fd = socket(AF_INET, SOCK_DGRAM, 0);
                if (s->host_fd >= 0) {
                    set_sock_nonblock(s->host_fd);
                    uint16_t src_port = ((uint16_t)s->regs[W6300_Sn_PORTR0] << 8) |
                                        s->regs[W6300_Sn_PORTR0 + 1];
                    if (src_port > 0) {
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
        } else if (mode == W6300_MR_MACRAW) {
            s->regs[W6300_Sn_SR] = W6300_SOCK_MACRAW;
            if (sock == 0)
                w6300_macraw_attach(dev, sock);
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
            if (dev->live && s->host_fd >= 0) {
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
#ifdef __EMSCRIPTEN__
            if (dev->live) {
                int wport = ((int)s->regs[W6300_Sn_PORTR0] << 8) | (int)s->regs[W6300_Sn_PORTR0 + 1];
                uint8_t msg[4];
                msg[0] = 0x4C; msg[1] = (uint8_t)(sock & 0xFF);
                msg[2] = (uint8_t)(wport & 0xFF); msg[3] = (uint8_t)((wport >> 8) & 0xFF);
                w6300_ws_tx_push(msg, 4);
            }
#endif
        }
        break;

    case W6300_CMD_CONNECT:
        if (s->regs[W6300_Sn_SR] == W6300_SOCK_INIT) {
            if (dev->live && s->host_fd >= 0) {
                struct sockaddr_in dest;
                w6300_build_addr(s, &dest);
                int rc = connect(s->host_fd, (struct sockaddr *)&dest, sizeof(dest));
                if (rc == 0) {
                    s->regs[W6300_Sn_SR] = W6300_SOCK_ESTABLISHED;
                } else if (errno == EINPROGRESS) {
                    s->regs[W6300_Sn_SR] = W6300_SOCK_SYNSENT;
                } else {
                    fprintf(stderr, "[W6300] Socket %d connect failed: %s\n",
                            sock, strerror(errno));
                    s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
                }
            } else {
                s->regs[W6300_Sn_SR] = W6300_SOCK_ESTABLISHED;
            }
#ifdef __EMSCRIPTEN__
            if (dev->live) {
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
#endif
        }
        break;

    case W6300_CMD_DISCON:
        if (s->regs[W6300_Sn_SR] == W6300_SOCK_ESTABLISHED) {
            s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
            if (dev->live) w6300_close_host_sock(s);
            s->regs[W6300_Sn_IR] |= W6300_IR_DISCON;
        }
        break;

    case W6300_CMD_CLOSE:
        s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
        if (dev->live) w6300_close_host_sock(s);
#ifdef __EMSCRIPTEN__
        if (dev->live) {
            uint8_t msg[2];
            msg[0] = 0x58; msg[1] = (uint8_t)(sock & 0xFF);
            w6300_ws_tx_push(msg, 2);
        }
#endif
        break;

    case W6300_CMD_SEND: {
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
            if (w6300_mode_is_udp(mode)) {
                struct sockaddr_in dest;
                w6300_build_addr(s, &dest);
                sendto(s->host_fd, send_buf, data_len, 0,
                       (struct sockaddr *)&dest, sizeof(dest));
            } else {
                send(s->host_fd, send_buf, data_len, MSG_NOSIGNAL);
            }
        }
#ifdef __EMSCRIPTEN__
        if (dev->live && data_len > 0) {
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
#endif
        s->regs[W6300_Sn_TX_RD0] = s->regs[W6300_Sn_TX_WR0];
        s->regs[W6300_Sn_TX_RD0 + 1] = s->regs[W6300_Sn_TX_WR0 + 1];
        s->regs[W6300_Sn_TX_FSR0] = (W6300_TX_BUF_SIZE >> 8) & 0xFF;
        s->regs[W6300_Sn_TX_FSR0 + 1] = W6300_TX_BUF_SIZE & 0xFF;
        s->regs[W6300_Sn_IR] |= W6300_IR_SENDOK;
        break;
    }

    case W6300_CMD_RECV:
        if (mode == W6300_MR_MACRAW && sock == 0 && dev->vnet_port >= 0) {
            uint16_t rx_rd = ((uint16_t)s->regs[W6300_Sn_RX_RD0] << 8) |
                             s->regs[W6300_Sn_RX_RD0 + 1];
            uint16_t rx_rsr = ((uint16_t)s->regs[W6300_Sn_RX_RSR0] << 8) |
                              s->regs[W6300_Sn_RX_RSR0 + 1];
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
    case W6300_CIDR0: return 0x61;
    case W6300_CIDR1: return 0x00;
    case W6300_CIDR2: return 0x11;
    case W6300_SYSR:  return w6300_sysr(dev);
    case W6300_PHYSR: return w6300_physr();
    case W6300_SIR: {
        uint8_t sir = 0;
        for (int i = 0; i < W6300_NUM_SOCKETS; i++)
            if (dev->sockets[i].regs[W6300_Sn_IR]) sir |= (uint8_t)(1u << i);
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
        return;  /* read-only */
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
        if (val & W6300_SLCR_ARP4)  dev->common[W6300_SLIR] |= 0x40;
        if (val & W6300_SLCR_PING4) dev->common[W6300_SLIR] |= 0x20;
        if (val & W6300_SLCR_RS)    dev->common[W6300_SLIR] |= 0x02;
        w6300_board_refresh_int();
        return;
    default:
        break;
    }
    /* Network registers need NET-unlock (SHAR/GAR/SUBR/SIPR). */
    if ((addr >= W6300_SHAR0 && addr < W6300_SHAR0 + 6) ||
        (addr >= W6300_GAR0 && addr < W6300_GAR0 + 4) ||
        (addr >= W6300_SUBR0 && addr < W6300_SUBR0 + 4) ||
        (addr >= W6300_SIPR0 && addr < W6300_SIPR0 + 4)) {
        if (dev->net_locked) return;
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
    if (addr == W6300_Sn_IR) return;           /* WO: clear via IRCLR */
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
                dev->sockets[sock].regs[W6300_Sn_SR] == W6300_SOCK_MACRAW)
                break;
            dev->sockets[sock].rx_buf[addr % W6300_RX_BUF_SIZE] = val;
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
 * Live Networking: Poll host sockets for incoming data
 * ======================================================================== */

void w6300_poll(w6300_t *dev) {
    if (!dev->live) return;

    for (int i = 0; i < W6300_NUM_SOCKETS; i++) {
        w6300_socket_t *s = &dev->sockets[i];

        if (s->host_listen_fd >= 0 &&
            s->regs[W6300_Sn_SR] == W6300_SOCK_LISTEN) {
            struct pollfd pfd = { .fd = s->host_listen_fd, .events = POLLIN };
            if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
                struct sockaddr_in client;
                socklen_t clen = sizeof(client);
                int cfd = accept(s->host_listen_fd, (struct sockaddr *)&client, &clen);
                if (cfd >= 0) {
                    set_sock_nonblock(cfd);
                    if (s->host_fd >= 0) close(s->host_fd);
                    s->host_fd = cfd;
                    s->regs[W6300_Sn_SR] = W6300_SOCK_ESTABLISHED;
                    uint32_t ip = ntohl(client.sin_addr.s_addr);
                    s->regs[W6300_Sn_DIPR0]     = (ip >> 24) & 0xFF;
                    s->regs[W6300_Sn_DIPR0 + 1] = (ip >> 16) & 0xFF;
                    s->regs[W6300_Sn_DIPR0 + 2] = (ip >>  8) & 0xFF;
                    s->regs[W6300_Sn_DIPR0 + 3] = ip & 0xFF;
                    uint16_t port = ntohs(client.sin_port);
                    s->regs[W6300_Sn_DPORTR0]     = (port >> 8) & 0xFF;
                    s->regs[W6300_Sn_DPORTR0 + 1] = port & 0xFF;
                    s->regs[W6300_Sn_IR] |= W6300_IR_CON;
                }
            }
        }

        if (s->host_fd >= 0 && s->regs[W6300_Sn_SR] == W6300_SOCK_SYNSENT) {
            struct pollfd pfd = { .fd = s->host_fd, .events = POLLOUT };
            if (poll(&pfd, 1, 0) > 0 &&
                (pfd.revents & (POLLOUT | POLLERR | POLLHUP))) {
                int err = 0;
                socklen_t elen = sizeof(err);
                getsockopt(s->host_fd, SOL_SOCKET, SO_ERROR, &err, &elen);
                if (err == 0) {
                    s->regs[W6300_Sn_SR] = W6300_SOCK_ESTABLISHED;
                    s->regs[W6300_Sn_IR] |= W6300_IR_CON;
                } else {
                    s->regs[W6300_Sn_SR] = W6300_SOCK_CLOSED;
                    close(s->host_fd);
                    s->host_fd = -1;
                }
            }
        }

        if (s->host_fd >= 0 &&
            (s->regs[W6300_Sn_SR] == W6300_SOCK_ESTABLISHED ||
             s->regs[W6300_Sn_SR] == W6300_SOCK_UDP)) {

            uint16_t rx_rsr = ((uint16_t)s->regs[W6300_Sn_RX_RSR0] << 8) |
                              s->regs[W6300_Sn_RX_RSR0 + 1];
            uint16_t free_space = W6300_RX_BUF_SIZE - rx_rsr;
            if (free_space == 0) continue;

            struct pollfd pfd = { .fd = s->host_fd, .events = POLLIN };
            if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
                uint16_t rx_wr = ((uint16_t)s->regs[W6300_Sn_RX_WR0] << 8) |
                                 s->regs[W6300_Sn_RX_WR0 + 1];

                uint8_t tmp[W6300_RX_BUF_SIZE];
                int is_udp = (s->regs[W6300_Sn_SR] == W6300_SOCK_UDP);
                uint8_t hdr[8];
                int hl = 0;
                ssize_t n = -1;
                if (is_udp) {
                    if (free_space > 8) {
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

static w6300_t w6300_board_dev_state;
static int w6300_board_on = 0;
static int w6300_board_spi_num = W6300_BOARD_SPI_DEFAULT;

static void w6300_board_refresh_int(void) {
    if (!w6300_board_on) return;
    int pending = 0;
    for (int i = 0; i < W6300_NUM_SOCKETS; i++) {
        if (w6300_board_dev_state.sockets[i].regs[W6300_Sn_IR]) {
            pending = 1;
            break;
        }
    }
    if (w6300_board_dev_state.common[W6300_IR]) pending = 1;
    gpio_set_direction(W6300_BOARD_INT_PIN, 0);
    gpio_mark_driven(W6300_BOARD_INT_PIN);
    gpio_set_input_pin(W6300_BOARD_INT_PIN, pending ? 0 : 1);
}

int w6300_board_enabled(void) { return w6300_board_on; }
int w6300_board_spi(void) { return w6300_board_spi_num; }
w6300_t *w6300_board_dev(void) { return &w6300_board_dev_state; }

void w6300_board_update_int(void) {
    if (!w6300_board_on) return;
    int pending = 0;
    for (int i = 0; i < W6300_NUM_SOCKETS; i++) {
        if (w6300_board_dev_state.sockets[i].regs[W6300_Sn_IR]) {
            pending = 1;
            break;
        }
    }
    if (w6300_board_dev_state.common[W6300_IR]) pending = 1;
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
    if (!w6300_board_on) return;
    w6300_board_on = 0;
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
