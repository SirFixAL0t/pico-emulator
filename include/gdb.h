/*
 * GDB Remote Serial Protocol Stub
 *
 * TCP server implementing GDB RSP for debugging firmware in the emulator.
 * Supports: register read/write, memory read/write, breakpoints,
 * watchpoints, conditional breakpoints, dual-core debugging,
 * single-step, continue.
 *
 * Usage: ./picoemu firmware.uf2 -gdb [port]
 * Then: arm-none-eabi-gdb -ex "target remote :3333"
 */

#ifndef GDB_H
#define GDB_H

#include <stdint.h>

/* Default GDB server port */
#define GDB_DEFAULT_PORT 3333

/* Limits */
#define GDB_MAX_BREAKPOINTS 16
#define GDB_MAX_WATCHPOINTS 16

/* Watchpoint types (matches GDB Z packet numbering) */
#define GDB_WP_WRITE   2   /* Z2: write watchpoint */
#define GDB_WP_READ    3   /* Z3: read watchpoint */
#define GDB_WP_ACCESS  4   /* Z4: access (read or write) watchpoint */

/* Conditional breakpoint condition types */
#define GDB_COND_NONE    0
#define GDB_COND_REG_EQ  1  /* register == value */
#define GDB_COND_REG_NE  2  /* register != value */
#define GDB_COND_REG_LT  3  /* register < value (unsigned) */
#define GDB_COND_REG_GT  4  /* register > value (unsigned) */
#define GDB_COND_MEM_EQ  5  /* *(uint32_t*)addr == value */
#define GDB_COND_MEM_NE  6  /* *(uint32_t*)addr != value */

/* Breakpoint condition */
typedef struct {
    int type;               /* GDB_COND_* */
    uint8_t reg_num;        /* For register conditions (0-15) */
    uint32_t addr;          /* For memory conditions */
    uint32_t value;         /* Compare value */
} gdb_condition_t;

/* Watchpoint entry */
typedef struct {
    uint32_t addr;
    uint32_t length;        /* Watch region size (1, 2, or 4 bytes) */
    int type;               /* GDB_WP_WRITE/READ/ACCESS */
    int active;
} gdb_watchpoint_t;

/* GDB server state */
typedef struct {
    int server_fd;              /* Listening socket */
    int client_fd;              /* Connected client socket */
    int port;                   /* TCP port */
    int active;                 /* Server is running */
    int single_step;            /* Single-step mode */

    /* Thread/core selection (Thread 1 = Core 0, Thread 2 = Core 1) */
    int g_thread;               /* Thread for register operations (0-based core ID) */
    int c_thread;               /* Thread for continue/step (-1 = all) */
    int stop_core;              /* Which core triggered the stop */

    /* Breakpoints */
    uint32_t breakpoints[GDB_MAX_BREAKPOINTS];
    int bp_active[GDB_MAX_BREAKPOINTS];
    gdb_condition_t bp_cond[GDB_MAX_BREAKPOINTS]; /* Per-breakpoint condition */
    int bp_count;

    /* Watchpoints */
    gdb_watchpoint_t watchpoints[GDB_MAX_WATCHPOINTS];
    int wp_count;
    int wp_hit;                 /* Watchpoint triggered flag */
    uint32_t wp_hit_addr;       /* Address that triggered */
    int wp_hit_type;            /* GDB_WP_WRITE/READ/ACCESS */

    /* Packet buffer */
    char pkt_buf[4096];
    int pkt_len;
} gdb_state_t;

extern gdb_state_t gdb;

/* ========================================================================
 * API
 * ======================================================================== */

/* Initialize and start GDB server (blocking until client connects) */
int gdb_init(int port);

/* Cleanup and close sockets */
void gdb_cleanup(void);

/* Check if execution should stop (breakpoint hit, single-step, or watchpoint)
 * core_id: which core to check (0 or 1) */
int gdb_should_stop(uint32_t pc, int core_id);

/* Handle GDB commands (called when execution stops) */
/* Returns: 0 = continue execution, 1 = single-step, -1 = detach/quit */
int gdb_handle(void);

#ifdef __EMSCRIPTEN__
/* WASM WebSocket transport (see src/gdb.c): non-blocking queues */
void picoemu_gdb_push_rx(const uint8_t *data, int len);
int picoemu_gdb_pop_tx(uint8_t *out, int maxlen);
int picoemu_gdb_tx_len(void);
int picoemu_gdb_start(void);
void picoemu_gdb_stop(void);
void picoemu_gdb_notify_stop(void);
/* Poll one packet while stopped: 0=resume,1=step,2=still stopped,-1=detach */
int picoemu_gdb_poll(void);
#endif

/* Watchpoint checks - called from membus on every memory access.
 * Return 1 if watchpoint triggered, 0 otherwise.
 * Gated internally: no-op when GDB inactive or no watchpoints set. */
int gdb_check_watchpoint_write(uint32_t addr, uint32_t size);
int gdb_check_watchpoint_read(uint32_t addr, uint32_t size);

/* RISC-V GDB support */
#include "rp2350_rv/rv_cpu.h"
extern rv_cpu_state_t *gdb_rv_harts[2];
extern int gdb_is_riscv;

#endif /* GDB_H */
