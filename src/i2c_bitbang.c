/*
 * GPIO bit-bang I2C slave bridge (see include/i2c_bitbang.h).
 *
 * One observer for all low pins; up to two concurrent trackers (one per
 * I2C bus in practice). ACKs are driven by forcing the SDA input latch
 * during the ACK clock only — when the guest drives the pin itself
 * (OE=1) the latch is invisible (effective level = OUT), so HW-style
 * traffic can never be corrupted, only open-drain release windows.
 *
 * Binding needs no guessing among idle-high pins: a START (SDA fall)
 * records a pending bus, and the tracker binds when a clock pin that
 * fell *after* that START rises for the first data clock. A conformant
 * master always lowers SCL before the first setup bit, so the
 * START-tick < SCL-fall-tick ordering is exact; unrelated GPIO that
 * happens to fall/rise can not satisfy it. Stale state expires by
 * SIO-write tick.
 */

#include "i2c_bitbang.h"
#include "i2c.h"
#include "gpio.h"
#include <stddef.h>

/* Tracker states */
#define BB_ADDR  0   /* shifting address byte */
#define BB_ACK   1   /* 9th clock pending (address or data) */
#define BB_HELD  2   /* ACK driven, awaiting falling edge to release */
#define BB_DATA  3   /* shifting master->slave data byte */
#define BB_READ  4   /* driving slave->master data bits */
#define BB_MACK  5   /* master-ACK clock after a read byte */
#define BB_END   6   /* NACKed read end: ignore clocks until STOP/RESTART */

#define BB_TRACKS 2
#define BB_PEND 4
#define BB_TICKS_PER_BIND 64u     /* START->first-clock window (SIO writes) */
#define BB_IDLE_EXPIRE 20000u     /* drop silent trackers */

typedef struct {
    int live;
    int scl, sda;
    int state;
    int after;          /* state to enter when BB_HELD releases */
    uint8_t cur;
    int bits;
    int addr, rw;
    int ack;            /* 1 = pull SDA low on the ACK clock */
    uint8_t out;
    int obits;
    void *dev_ctx;
    i2c_device_event_fn stop_fn;
    uint32_t last_act;
} bb_track_t;

static bb_track_t bb_tracks[BB_TRACKS];
static int bb_pend_sda[BB_PEND];
static uint32_t bb_pend_tick[BB_PEND];
static uint32_t bb_last_fall[32];
static uint32_t bb_tick = 0;

void i2c_bb_reset(void) {
    for (int i = 0; i < BB_TRACKS; i++) {
        if (bb_tracks[i].live)
            gpio_inject_level((uint8_t)bb_tracks[i].sda, 1);
        bb_tracks[i].live = 0;
    }
    for (int i = 0; i < BB_PEND; i++) bb_pend_sda[i] = -1;
    for (int i = 0; i < 32; i++) bb_last_fall[i] = 0;
    bb_tick = 0;
}

/* FUNCSEL field of IO_BANK0 GPIO_CTRL (low 5 bits). */
#define BB_FUNC_SIO 5

static int bb_pin_is_sio(int pin) {
    if (pin < 0 || pin >= 32) return 0;
    return (gpio_state.pins[pin].ctrl & 0x1Fu) == BB_FUNC_SIO;
}

static bb_track_t *bb_find_sda(int sda) {
    for (int i = 0; i < BB_TRACKS; i++)
        if (bb_tracks[i].live && bb_tracks[i].sda == sda)
            return &bb_tracks[i];
    return NULL;
}

static void bb_release(bb_track_t *t) {
    gpio_inject_level((uint8_t)t->sda, 1);
    t->live = 0;
}

static void bb_expire(void) {
    for (int i = 0; i < BB_TRACKS; i++) {
        if (bb_tracks[i].live && bb_tick - bb_tracks[i].last_act > BB_IDLE_EXPIRE)
            bb_release(&bb_tracks[i]);
    }
    for (int i = 0; i < BB_PEND; i++) {
        if (bb_pend_sda[i] >= 0 && bb_tick - bb_pend_tick[i] > BB_TICKS_PER_BIND)
            bb_pend_sda[i] = -1;
    }
}

/* Record a START fall. A repeat fall on the same pin is a fresh START. */
static void bb_pend_add(int sda) {
    for (int i = 0; i < BB_PEND; i++) {
        if (bb_pend_sda[i] == sda) {
            bb_pend_tick[i] = bb_tick;
            return;
        }
    }
    for (int i = 0; i < BB_PEND; i++) {
        if (bb_pend_sda[i] < 0) {
            bb_pend_sda[i] = sda;
            bb_pend_tick[i] = bb_tick;
            return;
        }
    }
    bb_pend_sda[0] = sda;
    bb_pend_tick[0] = bb_tick;
}

/* Most recent pending SDA whose START predates pin p's last fall
 * (i.e. p went low after the START: the conformant clock pattern).
 * Consumes and returns it, or -1. */
static int bb_pend_take_before(int p) {
    int best = -1;
    for (int i = 0; i < BB_PEND; i++) {
        int sda = bb_pend_sda[i];
        if (sda < 0 || sda == p) continue;
        if (bb_tick - bb_pend_tick[i] > BB_TICKS_PER_BIND) {
            bb_pend_sda[i] = -1;
            continue;
        }
        if (bb_pend_tick[i] >= bb_last_fall[(uint32_t)p]) continue;
        if (best < 0 || bb_pend_tick[i] > bb_pend_tick[(uint32_t)best])
            best = i;
    }
    if (best < 0) return -1;
    int sda = bb_pend_sda[best];
    bb_pend_sda[best] = -1;
    return sda;
}

static bb_track_t *bb_alloc(int scl, int sda) {
    for (int i = 0; i < BB_TRACKS; i++)
        if (!bb_tracks[i].live) {
            bb_tracks[i].live = 1;
            bb_tracks[i].scl = scl;
            bb_tracks[i].sda = sda;
            bb_tracks[i].state = BB_ADDR;
            bb_tracks[i].cur = 0;
            bb_tracks[i].bits = 0;
            bb_tracks[i].addr = 0;
            bb_tracks[i].rw = 0;
            bb_tracks[i].dev_ctx = NULL;
            bb_tracks[i].stop_fn = NULL;
            bb_tracks[i].last_act = bb_tick;
            return &bb_tracks[i];
        }
    return NULL;
}

/* Address byte complete: resolve the slave, run its start callback
 * (same as the DW RESTART path — jsmirror logs START|addr here),
 * decide the ACK, and queue the follow-up state. */
static void bb_addr_done(bb_track_t *t) {
    t->addr = (t->cur >> 1) & 0x7F;
    t->rw = t->cur & 1;
    t->ack = i2c_probe_ack((uint8_t)t->addr);
    t->dev_ctx = NULL;
    t->stop_fn = NULL;
    if (t->ack) {
        /* Mirror of find_device_any, but keep the callbacks so the
         * device model sees the same events as on the DW path. */
        for (int bus = 0; bus < 2; bus++) {
            for (int i = 0; i < i2c_state[bus].device_count; i++) {
                if (i2c_state[bus].devices[i].addr == (t->addr & 0x7F)) {
                    if (i2c_state[bus].devices[i].start_fn)
                        i2c_state[bus].devices[i].start_fn(
                            i2c_state[bus].devices[i].ctx);
                    t->dev_ctx = i2c_state[bus].devices[i].ctx;
                    t->stop_fn = i2c_state[bus].devices[i].stop_fn;
                    bus = 2;
                    break;
                }
            }
        }
        if (t->rw) {
            t->out = i2c_slave_read_byte((uint8_t)t->addr);
            t->obits = 0;
            t->after = BB_READ;
        } else {
            t->after = BB_DATA;
        }
    } else {
        t->after = BB_END;
    }
    t->state = BB_ACK;
}

static void bb_on_scl_rise(bb_track_t *t, uint32_t eff) {
    int sda_lv = (eff >> (uint32_t)t->sda) & 1u;
    t->last_act = bb_tick;
    switch (t->state) {
    case BB_ADDR:
    case BB_DATA:
        t->cur = (uint8_t)((t->cur << 1) | (uint8_t)sda_lv);
        if (++t->bits == 8) {
            if (t->state == BB_ADDR) {
                bb_addr_done(t);
            } else {
                /* Data byte to the slave (write_fn doubles as the
                 * jsmirror ring push, exactly like the DW path). */
                int rc = i2c_slave_write_byte((uint8_t)t->addr, t->cur);
                t->ack = (rc == 0);
                t->after = BB_DATA;
                t->state = BB_ACK;
            }
            t->cur = 0;
            t->bits = 0;
        }
        break;
    case BB_ACK:
        /* Drive the ACK clock: slave pulls SDA low for ACK. The latch
         * is forced only for this window and released on the fall. */
        if (t->ack) gpio_inject_level((uint8_t)t->sda, 0);
        t->state = BB_HELD;
        break;
    case BB_HELD:
        /* Malformed (rise while holding): release and drop. */
        bb_release(t);
        break;
    case BB_READ:
        gpio_inject_level((uint8_t)t->sda, (t->out & 0x80u) ? 1 : 0);
        t->out = (uint8_t)(t->out << 1);
        if (++t->obits == 8) t->state = BB_MACK;
        break;
    case BB_MACK:
        if (!sda_lv) {
            /* Master ACKed: next read byte. */
            t->out = i2c_slave_read_byte((uint8_t)t->addr);
            t->obits = 0;
            t->state = BB_READ;
        } else {
            t->state = BB_END;
        }
        break;
    case BB_END:
    default:
        break;
    }
}

static void bb_on_scl_fall(bb_track_t *t) {
    t->last_act = bb_tick;
    if (t->state == BB_HELD) {
        gpio_inject_level((uint8_t)t->sda, 1);
        t->state = t->after;
        if (t->state == BB_DATA) {
            t->cur = 0;
            t->bits = 0;
        }
    }
}

void i2c_bb_observe(uint32_t old_eff, uint32_t new_eff) {
    if (!i2c_any_attached()) return;
    uint32_t changed = (old_eff ^ new_eff) & 0xFFFFFFFFu;
    if (!changed) return;
    (void)old_eff;
    bb_tick++;
    bb_expire();

    /* Stamp falls first: binding order depends on them. */
    uint32_t c = changed;
    while (c) {
        int p = __builtin_ctz(c);
        c &= c - 1;
        if (!((new_eff >> (uint32_t)p) & 1u))
            bb_last_fall[p] = bb_tick;
    }

    /* SDA transitions (START/STOP/RESTART framing). */
    c = changed;
    while (c) {
        int d = __builtin_ctz(c);
        c &= c - 1;
        if (!bb_pin_is_sio(d)) continue;
        int rising = (new_eff >> (uint32_t)d) & 1u;
        if (rising) {
            /* STOP candidate: live tracker on this SDA whose SCL is high. */
            bb_track_t *t = bb_find_sda(d);
            if (t && ((new_eff >> (uint32_t)t->scl) & 1u)) {
                if (t->stop_fn && t->dev_ctx) t->stop_fn(t->dev_ctx);
                bb_release(t);
            }
            /* A rise alone never cancels a pending START: the first
             * setup bit of a scan is often 1 (released while SCL is
             * still low); expiry reaps genuine strays. */
        } else {
            /* SDA fall: RESTART on a live bus, else a pending START. */
            bb_track_t *t = bb_find_sda(d);
            if (t && ((new_eff >> (uint32_t)t->scl) & 1u)) {
                if (t->state == BB_END ||
                    ((t->state == BB_ADDR || t->state == BB_DATA) && t->bits == 0)) {
                    t->state = BB_ADDR;
                    t->cur = 0;
                    t->bits = 0;
                    t->dev_ctx = NULL;
                    t->stop_fn = NULL;
                    t->last_act = bb_tick;
                } else if (t->state == BB_ACK || t->state == BB_HELD ||
                           t->state == BB_MACK) {
                    /* RESTART replaces the pending ACK clock. */
                    gpio_inject_level((uint8_t)t->sda, 1);
                    t->state = BB_ADDR;
                    t->cur = 0;
                    t->bits = 0;
                    t->dev_ctx = NULL;
                    t->stop_fn = NULL;
                    t->last_act = bb_tick;
                } else {
                    bb_release(t);
                }
            } else if (!t) {
                /* Fresh START needs some other pin high (the idle
                 * clock); otherwise it is just a data line going low. */
                int any_high = 0;
                for (int p = 0; p < 32; p++) {
                    if (p == d) continue;
                    if ((new_eff >> (uint32_t)p) & 1u) { any_high = 1; break; }
                }
                if (any_high) bb_pend_add(d);
            }
        }
    }

    /* Clocks. */
    c = changed;
    while (c) {
        int p = __builtin_ctz(c);
        c &= c - 1;
        if (!bb_pin_is_sio(p)) continue;
        int rising = (new_eff >> (uint32_t)p) & 1u;
        int handled = 0;
        for (int i = 0; i < BB_TRACKS; i++) {
            if (!bb_tracks[i].live || bb_tracks[i].scl != p) continue;
            handled = 1;
            if (rising) bb_on_scl_rise(&bb_tracks[i], new_eff);
            else bb_on_scl_fall(&bb_tracks[i]);
        }
        if (rising && !handled) {
            /* First data clock of a pending bus binds (scl=p). */
            int sda = bb_pend_take_before(p);
            if (sda >= 0) {
                bb_track_t *t = bb_alloc(p, sda);
                if (t) bb_on_scl_rise(t, new_eff);
            }
        }
    }
}
