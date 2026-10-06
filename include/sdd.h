#ifndef SDD_H
#define SDD_H

#include <stdint.h>

/* ========================================================================
 * Pico-emu Software-Defined Device (SDD) Framework
 *
 * Pluggable virtual peripherals that attach to the emulator via I2C, SPI,
 * or the virtual network bus. Each SDD is a self-contained device model
 * with optional periodic tick, bus callbacks, and network RX.
 *
 * Built-in devices:
 *   thermometer   — TMP102-compatible I2C temperature sensor (addr 0x48)
 *   eeprom        — 24LC256-compatible I2C EEPROM, 32KB (addr 0x50)
 *
 * Usage:
 *   -sdd thermometer              Use default temperature (25.0 C)
 *   -sdd thermometer:temp=37.5    Set initial temperature
 *   -sdd thermometer:i2c=1        Attach to I2C1 instead of I2C0
 *   -sdd eeprom                   32KB EEPROM on I2C0 addr 0x50
 *   -sdd eeprom:file=eeprom.bin   File-backed persistence
 *
 * Custom devices can be added by implementing the sdd_device_t interface
 * and calling sdd_register().
 * ======================================================================== */

#define SDD_MAX_DEVICES    8
#define SDD_NAME_LEN      32
#define SDD_MAX_OPTS      64

/* Device interface */
typedef struct sdd_device {
    char name[SDD_NAME_LEN];

    /* I2C attachment (optional, -1 if not I2C) */
    int i2c_bus;                /* 0 or 1 */
    int i2c_addr;               /* 7-bit address, -1 if not I2C */
    int (*i2c_write)(void *ctx, uint8_t data);
    uint8_t (*i2c_read)(void *ctx);
    void (*i2c_start)(void *ctx);
    void (*i2c_stop)(void *ctx);

    /* SPI attachment (optional) */
    int spi_bus;                /* 0 or 1, -1 if not SPI */
    uint8_t (*spi_xfer)(void *ctx, uint8_t mosi);
    void (*spi_cs)(void *ctx, int active);

    /* Network attachment (optional) */
    void (*net_rx)(void *ctx, const uint8_t *frame, int len);
    uint8_t mac[6];             /* Virtual MAC, zero = none */

    /* Periodic tick (optional, called from main loop) */
    void (*tick)(void *ctx, uint32_t elapsed_us);

    /* Cleanup */
    void (*cleanup)(void *ctx);

    void *ctx;                  /* Device-specific state */
} sdd_device_t;

/* Global SDD registry */
typedef struct {
    sdd_device_t devices[SDD_MAX_DEVICES];
    int count;
} sdd_registry_t;

extern sdd_registry_t sdd_registry;

/* ======================================================================== */
/* Lifecycle                                                                 */
/* ======================================================================== */

/* Initialize the SDD registry */
void sdd_init(void);

/* Cleanup all registered devices */
void sdd_cleanup(void);

/* ======================================================================== */
/* Registration                                                              */
/* ======================================================================== */

/* Register a device. Automatically attaches to I2C/SPI bus and vnet.
 * Returns device index or -1 on error. */
int  sdd_register(sdd_device_t *dev);

/* ======================================================================== */
/* Main Loop                                                                 */
/* ======================================================================== */

/* Tick all registered devices */
void sdd_tick_all(uint32_t elapsed_us);

/* ======================================================================== */
/* Built-in Device Factories                                                 */
/* ======================================================================== */

/* Parse a -sdd argument and create the appropriate device.
 * Format: "type[:key=val,key=val,...]"
 * Returns 0 on success, -1 if type is unknown. */
int  sdd_create_from_arg(const char *arg);

/* Create a TMP102-compatible thermometer SDD */
int  sdd_create_thermometer(float initial_temp, int i2c_bus, int i2c_addr);

/* Create a 24LC256-compatible EEPROM SDD (file may be NULL) */
int  sdd_create_eeprom(int i2c_bus, int i2c_addr, const char *file);

/* Create JS-mirror slave(s): ACKs the given addresses
 * and mirrors every transaction into a JS-pollable ring. addrs[] holds up
 * to 8 seven-bit addresses sharing one ring (sdd_add resets per call). */
int  sdd_create_jsmirror(int i2c_bus, const int *addrs, int naddrs);

/* JS-mirror ring: polled by the JS host (stable ABI, also used by tests).
 * Entry encoding: 0x100|addr7 = START, 0x200 = STOP, 0x00-0xFF = byte. */
int  picoemu_jsmirror_pending(void);
int  picoemu_jsmirror_drops(void);
int  picoemu_jsmirror_pop(uint16_t *out, int max);
/* Clear ring + drop counter without detaching devices (reset paths). */
void jsmirror_reset(void);

/* Create an SPI-mirror slave (MOSI observe + MISO inject, see above).
 * Replaces any sdcard/emmc on that bus (single PL022 device slot). */
int  sdd_create_spimirror(int spi_bus);

/* SPI-mirror ring: 0x100|spi = CS assert, 0x200|spi = CS deassert,
 * 0x00-0xFF = MOSI byte. MISO replies come from the inject queue
 * (0xFF when empty). */
int  picoemu_spimirror_pending(void);
int  picoemu_spimirror_drops(void);
int  picoemu_spimirror_pop(uint16_t *out, int max);
int  picoemu_spimirror_inject(const uint8_t *data, int len);
void spimirror_reset(void);

#endif /* SDD_H */
