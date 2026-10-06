/*
 * SDD SPI-mirror device: observes MOSI and injects MISO for the
 * OpenHW-studio runner (MAX7219/SD-card cells).
 *
 * A virtual SPI slave: every transfer's MOSI byte is pushed into a
 * pollable ring, and the MISO reply comes from a JS-filled inject
 * queue (open-bus 0xFF when empty — same idle as the SD card model).
 * CS assert/deassert frame the ring. The simulator UI owns the actual
 * component models; this device is purely the electrical
 * termination + observer.
 *
 * Ring entry encoding (uint16_t):
 *   0x100 | spi   CS assert on bus spi (selected)
 *   0x200 | spi   CS deassert on bus spi
 *   0x00..0xFF    MOSI data byte
 *
 * NOTE: the PL022 model holds a single device slot per bus (last
 * attach wins), so a spimirror replaces any sdcard/emmc on that bus.
 * Overflow: drop-oldest + sticky drop counter (diagnostics).
 *
 * Usage:
 *   spimirror:spi=0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdd.h"

#define SPIMIRROR_RING_BITS  12
#define SPIMIRROR_RING_SIZE  (1u << SPIMIRROR_RING_BITS)
#define SPIMIRROR_RING_MASK  (SPIMIRROR_RING_SIZE - 1u)

#define SPIMIRROR_EVT_CS_ASSERT(spi)   (0x0100u | ((uint16_t)(spi) & 1u))
#define SPIMIRROR_EVT_CS_DEASSERT(spi) (0x0200u | ((uint16_t)(spi) & 1u))

#define SPIMIRROR_INJECT_SIZE 256

static uint16_t spimirror_ring[SPIMIRROR_RING_SIZE];
static uint32_t spimirror_head = 0;
static uint32_t spimirror_tail = 0;
static uint32_t spimirror_drops = 0;

static uint8_t spimirror_inject[SPIMIRROR_INJECT_SIZE];
static uint32_t spimirror_inj_head = 0;
static uint32_t spimirror_inj_tail = 0;

static int spimirror_last_cs[2] = { -1, -1 };

static void spimirror_push(uint16_t entry) {
    spimirror_ring[spimirror_head & SPIMIRROR_RING_MASK] = entry;
    spimirror_head++;
    if (spimirror_head - spimirror_tail > SPIMIRROR_RING_SIZE) {
        spimirror_tail = spimirror_head - SPIMIRROR_RING_SIZE;
        spimirror_drops++;
    }
}

/* --- device callbacks --- */

typedef struct {
    int spi_bus;
} sdd_spimirror_devctx_t;

static uint8_t spimirror_spi_xfer(void *ctx, uint8_t mosi) {
    (void)ctx;
    spimirror_push((uint16_t)mosi);
    if (spimirror_inj_tail != spimirror_inj_head) {
        uint8_t v = spimirror_inject[spimirror_inj_tail %
                                     SPIMIRROR_INJECT_SIZE];
        spimirror_inj_tail++;
        return v;
    }
    return 0xFF;  /* idle bus pulls high (matches sdcard model) */
}

static void spimirror_spi_cs(void *ctx, int cs_active) {
    sdd_spimirror_devctx_t *d = (sdd_spimirror_devctx_t *)ctx;
    if (!d || d->spi_bus < 0 || d->spi_bus > 1) return;
    if (spimirror_last_cs[d->spi_bus] == (cs_active ? 1 : 0)) return;
    spimirror_last_cs[d->spi_bus] = cs_active ? 1 : 0;
    spimirror_push(cs_active ? SPIMIRROR_EVT_CS_ASSERT(d->spi_bus)
                             : SPIMIRROR_EVT_CS_DEASSERT(d->spi_bus));
}

static void spimirror_cleanup(void *ctx) {
    free(ctx);
}

/* --- JS polling API (exported) --- */

void spimirror_reset(void) {
    spimirror_head = spimirror_tail = spimirror_drops = 0;
    spimirror_inj_head = spimirror_inj_tail = 0;
    spimirror_last_cs[0] = spimirror_last_cs[1] = -1;
}

int picoemu_spimirror_pending(void) {
    return (int)(spimirror_head - spimirror_tail);
}

int picoemu_spimirror_drops(void) {
    return (int)spimirror_drops;
}

int picoemu_spimirror_pop(uint16_t *out, int max) {
    int n = 0;
    if (!out || max <= 0) return 0;
    while (n < max && spimirror_tail != spimirror_head) {
        out[n++] = spimirror_ring[spimirror_tail & SPIMIRROR_RING_MASK];
        spimirror_tail++;
    }
    return n;
}

/* Queue MISO reply bytes for future transfers. Returns queued count. */
int picoemu_spimirror_inject(const uint8_t *data, int len) {
    int n = 0;
    if (!data || len <= 0) return 0;
    while (n < len &&
           spimirror_inj_head - spimirror_inj_tail < SPIMIRROR_INJECT_SIZE) {
        spimirror_inject[spimirror_inj_head % SPIMIRROR_INJECT_SIZE] = data[n];
        spimirror_inj_head++;
        n++;
    }
    return n;
}

/* --- factory --- */

int sdd_create_spimirror(int spi_bus) {
    if (spi_bus < 0 || spi_bus > 1) {
        fprintf(stderr, "[SDD] spimirror needs spi=0|1\n");
        return -1;
    }
    sdd_spimirror_devctx_t *state =
        (sdd_spimirror_devctx_t *)calloc(1, sizeof(sdd_spimirror_devctx_t));
    if (!state) return -1;
    state->spi_bus = spi_bus;

    sdd_device_t dev;
    memset(&dev, 0, sizeof(dev));
    strncpy(dev.name, "spimirror", SDD_NAME_LEN - 1);
    dev.i2c_bus = -1;
    dev.i2c_addr = -1;
    dev.spi_bus = spi_bus;
    dev.spi_xfer = spimirror_spi_xfer;
    dev.spi_cs = spimirror_spi_cs;
    dev.cleanup = spimirror_cleanup;
    dev.ctx = state;

    int idx = sdd_register(&dev);
    if (idx < 0) {
        free(state);
        return -1;
    }
    fprintf(stderr, "[SDD] spimirror on SPI%d (ring shared)\n", spi_bus);
    return idx;
}
