#ifndef I2C_BITBANG_H
#define I2C_BITBANG_H

#include <stdint.h>

/* ========================================================================
 * GPIO bit-bang I2C slave bridge
 *
 * MicroPython machine.I2C.scan() (and any zero-length probe) can not use
 * the DW_apb_i2c controller — the SDK forbids len == 0 — so MicroPython
 * falls back to soft-I2C: it wiggles SCL/SDA as open-drain GPIO and
 * samples SDA for the slave ACK. Without a GPIO-level slave the scan
 * always reports [] even with live slaves attached.
 *
 * This observer watches effective-pin transitions (hooked from the SIO
 * write path, ARM + RV32) for I2C START/STOP framing, decodes address +
 * data bytes, drives the ACK bit into the SDA input latch for matching
 * attached slaves, and routes bytes through the same device callbacks
 * the DW controller uses (start on address, write/read per byte, stop
 * on STOP) — so thermometer/EEPROM/jsmirror behave identically on both
 * paths. The jsmirror ring therefore also observes bit-banged traffic.
 *
 * Gating: observe() is a no-op unless an I2C device is attached on
 * either bus (i2c_any_attached), and a tracker only binds when both
 * pins are in SIO function — HW-SPI/PIO traffic can not bind. With no
 * sdd_add the board stays bit-identical.
 * ======================================================================== */

/* Feed an effective-pin transition (pins 0-31) into the observer. */
void i2c_bb_observe(uint32_t old_eff, uint32_t new_eff);

/* Clear tracker + edge-stamp state (called from i2c_init). */
void i2c_bb_reset(void);

#endif /* I2C_BITBANG_H */
