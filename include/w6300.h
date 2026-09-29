#ifndef W6300_H
#define W6300_H

#include <stdint.h>

/* ========================================================================
 * W6300 Ethernet Controller (QSPI Device Plugin)
 *
 * Emulates the WIZnet W6300 hardwired dual IPv4/IPv6 TCP/IP controller.
 * Same emulator role as the W5500 model (src/w5500.c): register-level
 * model for firmware development + MACRAW single-gateway path to the
 * shared vnet bus. No actual network I/O except live host sockets
 * (-net-live / -board-live) and the WASM proxy pump.
 *
 * Guest-visible differences vs W5500 (all from the W6300 datasheet
 * DS v1.0.2 + WIZnet ioLibrary_Driver Ethernet/W6300/w6300.h):
 *
 * 1. Host interface is QSPI SINGLE mode by default (same 4 wires as
 *    W5500 SPI: SCLK/CSn/MOSI/MISO). Frame per CS transaction:
 *      opcode(1B) = [block:5][R/W:1][QSPI-mode:2; 0x00=single]
 *      addr(2B, BE) | dummy(1B) | N data bytes
 *    opcode block: 0=common, 1+4N=socket N regs, 2+4N=TX buf,
 *    3+4N=RX buf (SAME encoding as W5500 BSB). R/W bit: 0=read,
 *    1=write (0x20). This model implements single mode; dual/quad
 *    need IO2/IO3 lines the PL022 SPI model has no wires for.
 * 2. 16-bit register addresses (not 8-bit offsets). Common regs live
 *    at 0x0000-0x420F (CIDR/SYSR/IR/SIMR/PHYSR/NET/SHAR/GAR/SUBR/
 *    SIPR/locks/RTR), socket N regs at 0x0000-0x022D within the
 *    socket-N block (Sn_MR=0x0000, Sn_CR=0x0010, Sn_IR=0x0020,
 *    Sn_IMR=0x0024, Sn_IRCLR=0x0028, Sn_SR=0x0030, Sn_PORTR=0x0114,
 *    Sn_DHAR=0x0118, Sn_DIPR=0x0120, Sn_DPORTR=0x0140, Sn_MSSR=0x0110,
 *    Sn_TTLR=0x0108, Sn_TX_BSR=0x0200, Sn_TX_FSR=0x0204, Sn_TX_RD=0x0208,
 *    Sn_TX_WR=0x020C, Sn_RX_BSR=0x0220, Sn_RX_RSR=0x0224,
 *    Sn_RX_RD=0x0228, Sn_RX_WR=0x022C).
 * 3. Chip ID: CIDR0=0x61 @0x0000, CIDR1=0x00 @0x0001, CIDR2=0x11
 *    (minor/version) @0x0004. Guests probe CIDR2 like VERSIONR.
 * 4. SYSR @0x2000: CHPL=bit7 NETL=bit6 PHYL=bit5 (all LOCKED=1 at
 *    reset), SPI=bit0. Unlock magic: CHPLCKR=0xCE, NETLCKR=0x3A,
 *    PHYLCKR=0x53 (any other value locks). SYCR0=0x2004 (RST bit7),
 *    SYCR1=0x2005 (IEN bit7). SHAR/GAR/SUBR/SIPR need NET-unlock;
 *    SYCR0/1 need CHIP-unlock; PHYCR0/1 need PHY-unlock.
 * 5. PHYSR @0x3000 replaces PHYCFGR with OPPOSITE speed/duplex
 *    polarity: LNK=bit0(1=up), SPD=bit1(1=10M,0=100M),
 *    DPX=bit2(1=half,0=full). PHYCR1 @0x301D reset default 0x40.
 * 6. Socket modes: TCP4/TCP=0x01 UDP4/UDP=0x02 IPRAW4=0x03
 *    MACRAW=0x07 TCP6=0x09 UDP6=0x0A TCPD=0x0D UDPD=0x0E.
 * 7. Socket commands: OPEN=0x01 LISTEN=0x02 CONNECT=0x04 DISCON=0x08
 *    CLOSE=0x10 SEND=0x20 SEND_KEEP=0x22 RECV=0x40 CONNECT6=0x84
 *    SEND6=0xA0. Status: CLOSED=0x00 INIT=0x13 LISTEN=0x14
 *    SYNSENT=0x15 ESTABLISHED=0x17 UDP=0x22 MACRAW=0x42 (same).
 *    IR: CON=0x01 DISCON=0x02 RECV=0x04 TIMEOUT=0x08 SEND_OK=0x10.
 *    Sn_IR is WO on silicon; Sn_IRCLR (0x0028) is W1C clear.
 *    Common IR/SIR are RO; IRCLR/SLIRCLR/SIMR/SLIR/IRCLR provided.
 * 8. TX/RX buffers: 4KB per socket per direction (W5500: 2KB).
 *    MACRAW RX keeps the W5500 2-byte BE length-prefix framing
 *    (prefix INCLUDES its 2 bytes), same RECV dual-rhythm (in-tree
 *    addr-0 bursts vs ioLibrary RX_RD-advance commits).
 *
 * Attaches to an SPI bus via spi_attach_device().
 * ======================================================================== */

#define W6300_NUM_SOCKETS   8
#define W6300_TX_BUF_SIZE   4096    /* Per socket (W5500: 2048) */
#define W6300_RX_BUF_SIZE   4096    /* Per socket (W5500: 2048) */

/* QSPI opcode: [QSPI-mode:7..6][R/W:bit5][block:4..0].
 * From ioLibrary w6300.h: _W6300_SPI_WRITE_ = (0x01 << 5) = 0x20,
 * block = AddrSel & 0xFF (WIZCHIP_SREG_BLOCK(N) = 1+4N, NOT shifted).
 * So opcode = block | 0x20 (write) | QSPI-mode, e.g. sock-0 regs write
 * = 0x01|0x20 = 0x21 (single) or 0xA1 (quad), common write = 0x20/0xA0.
 * The Arduino driver uses QSPI_QUAD_MODE (0x80): opcodes arrive as
 * 0x80+block(+0x20). The model masks the mode bits, so single/dual/quad
 * all decode identically (byte path has no IO2/IO3 wires).
 * (DIFFERENT from W5500: W5500 puts block at bits [7..3] and R/W at
 * bit 2, so sock-0 regs write = (1<<3)|0x04 = 0x0C.) */
#define W6300_OP_BLOCK(op)  ((op) & 0x1F)
#define W6300_OP_WRITE(op)  (((op) >> 5) & 0x01)
#define W6300_OP_MODE(op)   ((op) & 0xC0)
#define W6300_OPCODE(bsb, wr) ((uint8_t)((bsb) | ((wr) ? 0x20 : 0x00)))
/* Socket-N register block number (ioLibrary WIZCHIP_SREG_BLOCK(N)=1+4N) */
#define W6300_SREG_BLK(n)   (1 + 4 * (n))
#define W6300_TXBUF_BLK(n)  (2 + 4 * (n))
#define W6300_RXBUF_BLK(n)  (3 + 4 * (n))

/* Opcode block mapping (same encoding as W5500 BSB) */
#define W6300_BLK_COMMON    0

/* Common register addresses (16-bit) */
#define W6300_CIDR0     0x0000  /* Chip ID major (RO, 0x61; Arduino VERSIONR alias) */
#define W6300_CIDR1     0x0001  /* Chip ID (RO, 0x00) */
#define W6300_CIDR2     0x0004  /* Minor/version (RO, 0x11) */
#define W6300_VERSIONR  W6300_CIDR0 /* Arduino lwIP_w6300 name for CIDR0 */
#define W6300_VERSIONR_VAL 0x61 /* W6300 version value (== CIDR0) */
#define W6300_SYSR      0x2000  /* System status (RO) */
#define W6300_SYCR0     0x2004  /* System config 0 (WO, RST bit7) */
#define W6300_SYCR1     0x2005  /* System config 1 (IEN bit7) */
#define W6300_IR        0x2100  /* Interrupt (RO; clear via IRCLR) */
#define W6300_SIR       0x2101  /* Socket interrupt (RO, computed) */
#define W6300_SLIR      0x2102  /* Socket-less interrupt (RO) */
#define W6300_IMR       0x2104  /* Interrupt mask */
#define W6300_IRCLR     0x2108  /* IR clear (WO, W1C) */
#define W6300_SIMR      0x2114  /* Socket interrupt mask */
#define W6300_SLIMR     0x2124  /* Socket-less interrupt mask */
#define W6300_SLIRCLR   0x2128  /* SLIR clear (WO, W1C) */
#define W6300_SLCR      0x2130  /* Socket-less command (ARP4/PING4/RS/...) */
#define W6300_PHYSR     0x3000  /* PHY status (RO) */
#define W6300_PHYCR0    0x301C  /* PHY control 0 (WO, PHY-unlock) */
#define W6300_PHYCR1    0x301D  /* PHY control 1 (reset 0x40, PHY-unlock) */
#define W6300_NETMR     0x4008  /* Network mode (IP4B bit0, IP6B bit1) */
#define W6300_NETMR2    0x4009  /* Network mode 2 (PPPoE bit0, DHAS bit7) */
#define W6300_NET4MR    0x4000  /* Network IPv4 mode (UNRB/PARP/RSTB/PB) */
#define W6300_NET6MR    0x4004  /* Network IPv6 mode (UNRB/PARP/RSTB/PB) */
#define W6300_SHAR0     0x4120  /* Source MAC (6B, NET-unlock) */
#define W6300_GAR0      0x4130  /* Gateway IPv4 (4B, NET-unlock) */
#define W6300_SUBR0     0x4134  /* Subnet mask (4B, NET-unlock) */
#define W6300_SIPR0     0x4138  /* Source IPv4 (4B, NET-unlock) */
#define W6300_LLAR0     0x4140  /* Link-local IPv6 (16B, NET-unlock) */
#define W6300_GUAR0     0x4150  /* Global unicast IPv6 (16B, NET-unlock) */
#define W6300_SUB6R0    0x4160  /* IPv6 subnet prefix (16B, NET-unlock) */
#define W6300_GA6R0     0x4170  /* IPv6 gateway (16B) */
#define W6300_SLDIP6R0  0x4180  /* Socket-less dest IPv6 (16B) */
#define W6300_SLDIPR0   0x418C  /* Socket-less dest IPv4 (4B) */
#define W6300_SLDHAR0   0x4190  /* Socket-less dest MAC (6B, RO) */
#define W6300_PINGIDR   0x4198  /* Socket-less PING ID */
#define W6300_PINGSEQR0 0x419C  /* Socket-less PING sequence (2B) */
#define W6300_UIPR0     0x41A0  /* Unreachable IPv4 (4B, RO) */
#define W6300_UPORTR0   0x41A4  /* Unreachable port (2B, RO) */
#define W6300_UIP6R0    0x41B0  /* Unreachable IPv6 (16B, RO) */
#define W6300_UPORT6R0  0x41C0  /* Unreachable IPv6 port (2B, RO) */
#define W6300_INTPTMR0  0x41C5  /* INT pending time (2B) */
#define W6300_PLR       0x41D0  /* RA prefix length (RO) */
#define W6300_PFR       0x41D4  /* RA prefix flags (RO) */
#define W6300_VLTR0     0x41D8  /* RA valid lifetime (4B, RO) */
#define W6300_PLTR0     0x41DC  /* RA preferred lifetime (4B, RO) */
#define W6300_PAR0      0x41E0  /* RA prefix address (16B, RO) */
#define W6300_ICMP6BLKR 0x41F0  /* ICMPv6 block register */
#define W6300_CHPLCKR   0x41F4  /* Chip lock (WO; 0xCE=unlock) */
#define W6300_NETLCKR   0x41F5  /* Net lock (WO; 0x3A=unlock) */
#define W6300_PHYLCKR   0x41F6  /* PHY lock (WO; 0x53=unlock) */
#define W6300_RTR0      0x4200  /* Retry time (2B) */
#define W6300_RCR       0x4204  /* Retry count */
#define W6300_SLRTR0    0x4208  /* Socket-less retry time (2B) */
#define W6300_SLRCR     0x420C  /* Socket-less retry count */
#define W6300_SLHOPR    0x420F  /* Socket-less hop limit */
#define W6300_TCNTR0    0x2016  /* Ticker counter (2B, RO) */
#define W6300_TCNTRCLR  0x2020  /* Ticker counter clear (WO) */
#define W6300_SLPSR     0x212C  /* Socket-less prefer source (AUTO/LLA/GUA) */

/* NETMR bits (ioLibrary NETMR_ANB/M6B/WOL/IP6B/IP4B) */
#define W6300_NETMR_IP4B (1u << 0)  /* 1 = block IPv4 packets */
#define W6300_NETMR_IP6B (1u << 1)  /* 1 = block IPv6 packets */
#define W6300_NETMR_WOL  (1u << 2)  /* Wake-on-LAN over UDP */
#define W6300_NETMR_M6B  (1u << 4)  /* Block IPv6 multicast PING */
#define W6300_NETMR_ANB  (1u << 5)  /* Block IPv6 all-node PING */
/* NET4MR/NET6MR bits (ioLibrary NETxMR_UNRB/PARP/RSTB/PB) */
#define W6300_NETXMR_PB   (1u << 0) /* Ping block */
#define W6300_NETXMR_RSTB (1u << 1) /* TCP RST block */
#define W6300_NETXMR_PARP (1u << 2) /* ARP before PING reply */
#define W6300_NETXMR_UNRB (1u << 3) /* Unreachable block */
/* NETMR2 bits (ioLibrary NETMR2_DHAS/PPPoE) */
#define W6300_NETMR2_PPPoE (1u << 0)
#define W6300_NETMR2_DHAS  (1u << 7) /* ARP-reply DST MAC select */

/* SYSR bits */
#define W6300_SYSR_CHPL (1u << 7)   /* 1 = chip regs locked */
#define W6300_SYSR_NETL (1u << 6)   /* 1 = net regs locked */
#define W6300_SYSR_PHYL (1u << 5)   /* 1 = PHY regs locked */
#define W6300_SYSR_SPI  (1u << 0)   /* 1 = SPI host interface */
/* Lock magic */
#define W6300_CHIP_UNLOCK   0xCE
#define W6300_NET_UNLOCK    0x3A
#define W6300_PHY_UNLOCK    0x53
/* SYCR bits */
#define W6300_SYCR0_RST (1u << 7)
#define W6300_SYCR1_IEN (1u << 7)

/* PHYSR bits (note OPPOSITE polarity vs W5500 PHYCFGR) */
#define W6300_PHYSR_LNK (1u << 0)   /* 1 = link up */
#define W6300_PHYSR_SPD (1u << 1)   /* 1 = 10M, 0 = 100M */
#define W6300_PHYSR_DPX (1u << 2)   /* 1 = half, 0 = full */

/* Common IR bits (ioLibrary IR_WOL/UNR6/IPCONF/UNR4/PTERM) */
#define W6300_IR_PTERM  (1u << 0) /* PPPoE terminated */
#define W6300_IR_UNR4   (1u << 1) /* Dest port unreachable (IPv4) */
#define W6300_IR_IPCONF (1u << 2) /* SIPR conflict */
#define W6300_IR_UNR6   (1u << 4) /* Dest port unreachable (IPv6) */
#define W6300_IR_WOL    (1u << 7) /* Wake-on-LAN */

/* Socket-less interrupt bits (ioLibrary SLIR_TOUT/ARP4/PING4/ARP6/...) */
#define W6300_SLIR_RA    (1u << 0)
#define W6300_SLIR_RS    (1u << 1)
#define W6300_SLIR_NS    (1u << 2)
#define W6300_SLIR_PING6 (1u << 3)
#define W6300_SLIR_ARP6  (1u << 4)
#define W6300_SLIR_PING4 (1u << 5)
#define W6300_SLIR_ARP4  (1u << 6)
#define W6300_SLIR_TOUT  (1u << 7)

/* Socket-less commands (SLCR) */
#define W6300_SLCR_ARP4     (1u << 6)
#define W6300_SLCR_PING4    (1u << 5)
#define W6300_SLCR_ARP6     (1u << 4)
#define W6300_SLCR_PING6    (1u << 3)
#define W6300_SLCR_NS       (1u << 2)
#define W6300_SLCR_RS       (1u << 1)
#define W6300_SLCR_UNA      (1u << 0)
/* Socket-less prefer-source values (SLPSR / Sn_PSR: AUTO/LLA/GUA) */
#define W6300_PSR_AUTO 0x00
#define W6300_PSR_LLA  0x02
#define W6300_PSR_GUA  0x03

/* Socket register addresses (within socket-N block) */
#define W6300_Sn_MR     0x0000
#define W6300_Sn_PSR    0x0004  /* Prefer source IPv6 (AUTO/LLA/GUA) */
#define W6300_Sn_CR     0x0010
#define W6300_Sn_IR     0x0020
#define W6300_Sn_IMR    0x0024
#define W6300_Sn_IRCLR  0x0028
#define W6300_Sn_SR     0x0030
#define W6300_Sn_ESR    0x0031  /* Extension status (RO: TCPM/TCPOP/IP6T) */
#define W6300_Sn_PNR    0x0100  /* IP protocol number (IPRAW) */
#define W6300_Sn_PORTR0 0x0114  /* Source port (2B BE) */
#define W6300_Sn_DHAR0  0x0118  /* Dest MAC (6B) */
#define W6300_Sn_DIPR0  0x0120  /* Dest IPv4 (4B) */
#define W6300_Sn_DPORTR0 0x0140 /* Dest port (2B BE) */
#define W6300_Sn_MSSR0  0x0110  /* Max segment size (2B) */
#define W6300_Sn_TOSR   0x0104
#define W6300_Sn_TTLR   0x0108
#define W6300_Sn_FRGR0  0x010C  /* Fragment offset (2B) */
#define W6300_Sn_DIP6R0 0x0130  /* Dest IPv6 (16B) */
#define W6300_Sn_MR2    0x0144  /* Socket mode 2 (DHAM/FARP) */
#define W6300_Sn_RTR0   0x0180  /* Socket retry time (2B) */
#define W6300_Sn_RCR    0x0184  /* Socket retry count */
#define W6300_Sn_KPALVTR 0x0188 /* Keep-alive timer */
#define W6300_Sn_TX_BSR 0x0200  /* TX buffer size reg */
#define W6300_Sn_TX_FSR0 0x0204 /* TX free size (2B RO) */
#define W6300_Sn_TX_RD0 0x0208  /* TX read pointer (2B RO) */
#define W6300_Sn_TX_WR0 0x020C  /* TX write pointer (2B) */
#define W6300_Sn_RX_BSR 0x0220  /* RX buffer size reg */
#define W6300_Sn_RX_RSR0 0x0224 /* RX received size (2B RO) */
#define W6300_Sn_RX_RD0 0x0228  /* RX read pointer (2B) */
#define W6300_Sn_RX_WR0 0x022C  /* RX write pointer (2B RO) */

/* Socket register block span (largest used offset + 2) */
#define W6300_SOCKET_REG_SIZE   0x0230

/* Socket status values (ioLibrary SOCK_*) */
#define W6300_SOCK_CLOSED   0x00
#define W6300_SOCK_INIT     0x13
#define W6300_SOCK_LISTEN   0x14
#define W6300_SOCK_SYNSENT  0x15
#define W6300_SOCK_SYNRECV  0x16
#define W6300_SOCK_ESTABLISHED 0x17
#define W6300_SOCK_FIN_WAIT 0x18
#define W6300_SOCK_CLOSING  0x1A
#define W6300_SOCK_TIME_WAIT 0x1B
#define W6300_SOCK_CLOSE_WAIT 0x1C
#define W6300_SOCK_LAST_ACK 0x1D
#define W6300_SOCK_UDP      0x22
#define W6300_SOCK_IPRAW4   0x32
#define W6300_SOCK_IPRAW6   0x33
#define W6300_SOCK_MACRAW   0x42

/* Socket commands (ioLibrary Sn_CR_*) */
#define W6300_CMD_OPEN      0x01
#define W6300_CMD_LISTEN    0x02
#define W6300_CMD_CONNECT   0x04
#define W6300_CMD_DISCON    0x08
#define W6300_CMD_CLOSE     0x10
#define W6300_CMD_SEND      0x20
#define W6300_CMD_SEND_MAC  0x21    /* SEND with MAC (skip ARP; UDP only) */
#define W6300_CMD_SEND_KEEP 0x22
#define W6300_CMD_RECV      0x40
#define W6300_CMD_CONNECT6  0x84
#define W6300_CMD_SEND6     0xA0

/* Socket modes */
#define W6300_MR_CLOSE  0x00
#define W6300_MR_TCP    0x01    /* TCP4 */
#define W6300_MR_UDP    0x02    /* UDP4 */
#define W6300_MR_IPRAW  0x03    /* IPRAW4 */
#define W6300_MR_MACRAW 0x07
#define W6300_MR_TCP6   0x09
#define W6300_MR_UDP6   0x0A
#define W6300_MR_IPRAW6 0x0B
#define W6300_MR_TCPD   0x0D
#define W6300_MR_UDPD   0x0E
/* Sn_MR2 bits (ioLibrary Sn_MR2_DHAM/FARP) */
#define W6300_MR2_FARP  (1u << 0)
#define W6300_MR2_DHAM  (1u << 1)
/* Sn_ESR bits (ioLibrary Sn_ESR_TCPM/TCPOP/IP6T) */
#define W6300_ESR_TCPM  (1u << 2) /* 1 = IPv6 TCP */
#define W6300_ESR_TCPOP (1u << 1) /* 1 = client */
#define W6300_ESR_IP6T  (1u << 0) /* 1 = GUA, 0 = LLA */

/* Socket IR bits */
#define W6300_IR_CON    0x01
#define W6300_IR_DISCON 0x02
#define W6300_IR_RECV   0x04
#define W6300_IR_TIMEOUT 0x08
#define W6300_IR_SENDOK 0x10

/* QSPI frame phase */
typedef enum {
    W6300_PHASE_OPCODE,   /* Opcode byte (block + R/W + mode) */
    W6300_PHASE_ADDR_HI,  /* Address byte 1 (BE) */
    W6300_PHASE_ADDR_LO,  /* Address byte 2 */
    W6300_PHASE_DUMMY,    /* Dummy clock byte (no MISO data) */
    W6300_PHASE_DATA,     /* Data transfer phase */
} w6300_phase_t;

/* Per-socket state (mirrors w5500_socket_t ring semantics) */
typedef struct {
    uint8_t regs[W6300_SOCKET_REG_SIZE];
    uint8_t tx_buf[W6300_TX_BUF_SIZE];
    uint8_t rx_buf[W6300_RX_BUF_SIZE];
    uint16_t rx_base;       /* Stream head (absolute ring offset) */
    uint16_t rx_cursor_base;/* VDM address of current CS frame's first byte */
    uint8_t  rx_cursor_valid;
    uint16_t tx_dirty_base;
    uint16_t tx_dirty_len;
    uint8_t  tx_dirty_valid;
    int     host_fd;
    int     host_listen_fd;
    uint32_t retry_ticks;    /* SYNSENT poll ticks since CONNECT (retry engine) */
    uint32_t kpalv_ticks;   /* ESTABLISHED poll ticks since last TX (KPALVTR) */
} w6300_socket_t;

/* W6300 device state */
typedef struct {
    /* Common registers: sparse 16-bit space, kept as flat array over
     * 0x0000..0x420F (0x4210 bytes: highest decoded reg is SLHOPR @
     * 0x420F). Only decoded offsets are live; the rest reads 0.
     * ~17KB per device, allocated statically. */
    uint8_t common[0x4210];

    w6300_socket_t sockets[W6300_NUM_SOCKETS];

    /* QSPI frame state machine */
    w6300_phase_t phase;
    uint8_t  opcode;
    uint8_t  bsb;
    uint16_t addr;
    int      rw;
    int      cs_active;

    int      live;
    int      vnet_port;
    /* Lock state mirrors SYSR (all locked at reset) */
    uint8_t  chip_locked;
    uint8_t  net_locked;
    uint8_t  phy_locked;
} w6300_t;

void w6300_init(w6300_t *dev);
uint8_t w6300_spi_xfer(void *ctx, uint8_t mosi);
void w6300_spi_cs(void *ctx, int cs_active);
void w6300_poll(w6300_t *dev);
void w6300_set_live(w6300_t *dev, int enable);

/* PIO-QSPI bridge (Arduino W6300 driver path): the driver moves bytes
 * with PIO SM + DMA (NOT the PL022 SPI): DMA feeds the QSPI command +
 * payload bytes into the SM's TXF, and pulls MISO bytes out of RXF.
 * These hooks snoop that traffic at the PIO register layer and feed the
 * same byte stream into the w6300 register/buffer model, so the driver
 * works unmodified. Write path: every word pushed to a TXF that belongs
 * to an armed QSPI SM is forwarded byte-wise (LSB first, matching the
 * driver's DMA_SIZE_8 + bswap layout). Read path: when the parser sits
 * in the DATA phase of a READ frame, each RXF read generates one model
 * byte with address auto-increment (real QSPI reads clock MISO out with
 * no further TX DATA bytes, so there is nothing to pre-queue); served
 * instead of the (empty) hardware FIFO.
 *
 * A QSPI SM is "armed" when the pico-w6300 board is on and the SM's
 * OUT base pin is 18 (IO0), i.e. the wiznet_pio_qspi program's OUT pins
 * 18..21 configuration. CS framing comes from the existing GPIO16 watch
 * (w6300_board_gpio_write -> w6300_spi_cs); the bridge forwards bytes
 * only while CS is asserted. Frame-start (CS assert) resets the bridge
 * to OPCODE phase, mirroring w6300_spi_cs.
 *
 * Zero cost when off: a single w6300_board_enabled() flag test on TXF
 * push / RXF pop. */
void w6300_pio_tx_write(int pio_num, int sm, uint32_t val);
uint32_t w6300_pio_rx_read(int pio_num, int sm);
int w6300_pio_rx_ready(int pio_num, int sm);
int w6300_pio_exec_out_drop(int pio_num, int sm);
void w6300_pio_sm_restart(int pio_num, int sm);
/* PIO register-layer hooks (called from pio.c, cheap when off) */
void w6300_pio_pinctrl(int pio_num, int sm);
int w6300_pio_is_armed(int pio_num, int sm);
int w6300_pio_rx_level(int pio_num, int sm);
void w6300_pio_cs_assert(void);

/* MACRAW single-gateway path (same vnet bus as W5500/CYW43) */
int w6300_macraw_attach(w6300_t *dev, int sock);
void w6300_gw_enable_set(int on);
int w6300_gw_enabled(void);

/* ========================================================================
 * W6300-EVB-Pico board variant (RP2040; -board pico-w6300)
 * W6300-EVB-Pico2 board variant (RP2350; -board pico-w6300-2)
 *
 * Real W6300-EVB-Pico wiring (WIZnet docs): QSPI on GPIO15-22 —
 *   INTn=GPIO15, CSn=GPIO16, SCLK=GPIO17, IO0/MOSI=GPIO18,
 *   IO1/MISO=GPIO19, IO2=GPIO20, IO3=GPIO21, RSTn=GPIO22.
 * Single-SPI mode uses the same 4 wires as W5500 (SCLK/CS/MOSI/MISO),
 * so the emulator serves it through the normal PL022 SPI path; the
 * board watches CSn=16/RSTn=22 (cf. pico-eth CSn=17/RSTn=20) and
 * drives INTn=GPIO15 active-low (cf. pico-eth GPIO21).
 * ======================================================================== */

#define W6300_BOARD_SPI_DEFAULT  0
#define W6300_BOARD_CS_PIN       16
#define W6300_BOARD_RST_PIN      22
#define W6300_BOARD_INT_PIN      15

void w6300_board_attach(int spi_num, int live);
void w6300_board_detach(void);
void w6300_board_reattach(void);
int w6300_board_enabled(void);
int w6300_board_spi(void);
w6300_t *w6300_board_dev(void);
void w6300_board_set_live(int live);
void w6300_board_gpio_write(uint32_t pin, uint32_t value);
void w6300_board_update_int(void);
void w6300_board_poll(void);

#endif /* W6300_H */
