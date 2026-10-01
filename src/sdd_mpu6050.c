/*
 * Software-Defined MPU-6050 6-DoF IMU
 *
 * I2C 6-axis MotionTracking device (3-axis Accel + 3-axis Gyro + Temp).
 * Default I2C address: 0x68 (AD0 low) or 0x69 (AD0 high).
 *
 * Register Map: 128 8-bit registers (0x00 to 0x7F).
 * Key Registers:
 *    0x3B - 0x40: ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT (6 bytes)
 *    0x41 - 0x42: TEMP_OUT (2 bytes)
 *    0x43 - 0x48: GYRO_XOUT,  GYRO_YOUT,  GYRO_ZOUT  (6 bytes)
 *    0x6B:        PWR_MGMT_1 (Device reset/sleep control)
 *    0x75:        WHO_AM_I   (Always returns 0x68)
 *
 * Protocol:
 *    Write register pointer:  [reg] + STOP
 *    Write register payload:  [reg] [data...] + STOP (auto-increments reg)
 *    Burst read:              [reg] + RESTART, then reads (auto-increments reg)
 *
 * Usage:
 *    sdd_create_mpu6050(0, 0x68, NULL);
 *    -sdd mpu6050                       I2C0 addr 0x68
 *    -sdd mpu6050:i2c=1,addr=0x69       Custom bus / AD0 high
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdd.h"

#define MPU6050_REG_COUNT     128u
#define MPU6050_REG_MASK      (MPU6050_REG_COUNT - 1u)

#define REG_ACCEL_XOUT_H      0x3B
#define REG_ACCEL_ZOUT_H      0x3F
#define REG_TEMP_OUT_H        0x41
#define REG_GYRO_XOUT_H       0x43
#define REG_PWR_MGMT_1        0x6B
#define REG_WHO_AM_I          0x75

#define MPU6050_DEVICE_ID     0x68

typedef struct {
    uint8_t regs[MPU6050_REG_COUNT];
    uint8_t reg_addr;  /* Latched register pointer */
    int     phase;     /* 0 = expect register address, 1 = writing data bytes */
} sdd_mpu6050_state_t;

/* Reset registers to datasheet power-on defaults */
static void mpu6050_reset_registers(sdd_mpu6050_state_t *s) {
    memset(s->regs, 0, sizeof(s->regs));

    /* Hardware Identification */
    s->regs[REG_WHO_AM_I] = MPU6050_DEVICE_ID;

    /* Datasheet reset default for PWR_MGMT_1 is 0x40 (Sleep mode enabled) */
    s->regs[REG_PWR_MGMT_1] = 0x40;

    /* Set default 1.0g on Accel Z-axis (assumes +/-2g full-scale -> 16384 LSB/g = 0x4000)
     * This ensures rest/gravity calculations in your payload setup evaluate normally */
    s->regs[REG_ACCEL_ZOUT_H]     = 0x40;
    s->regs[REG_ACCEL_ZOUT_H + 1] = 0x00;
}

static void mpu6050_i2c_start(void *ctx) {
    sdd_mpu6050_state_t *s = (sdd_mpu6050_state_t *)ctx;
    /* RESTART: Freeze latched register pointer for the upcoming burst read */
    s->phase = 0;
}

static void mpu6050_i2c_stop(void *ctx) {
    sdd_mpu6050_state_t *s = (sdd_mpu6050_state_t *)ctx;
    /* Transaction complete; next write will target a new register address */
    s->phase = 0;
}

static int mpu6050_i2c_write(void *ctx, uint8_t data) {
    sdd_mpu6050_state_t *s = (sdd_mpu6050_state_t *)ctx;

    if (s->phase == 0) {
        /* First byte of write transaction sets the internal register pointer */
        s->reg_addr = data & MPU6050_REG_MASK;
        s->phase = 1;
    } else {
        /* Subsequent bytes write into sensor registers */
        s->regs[s->reg_addr] = data;

        /* If firmware writes to PWR_MGMT_1 with DEVICE_RESET (bit 7 set) */
        if (s->reg_addr == REG_PWR_MGMT_1 && (data & 0x80)) {
            mpu6050_reset_registers(s);
        }

        /* Auto-increment register address per I2C standard burst write */
        s->reg_addr = (s->reg_addr + 1u) & MPU6050_REG_MASK;
    }
    return 0; /* ACK */
}

static uint8_t mpu6050_i2c_read(void *ctx) {
    sdd_mpu6050_state_t *s = (sdd_mpu6050_state_t *)ctx;

    /* Read byte from current register */
    uint8_t value = s->regs[s->reg_addr];

    /* Auto-increment register pointer for continuous 14-byte telemetry reads */
    s->reg_addr = (s->reg_addr + 1u) & MPU6050_REG_MASK;

    return value;
}

static void mpu6050_cleanup(void *ctx) {
    sdd_mpu6050_state_t *s = (sdd_mpu6050_state_t *)ctx;
    free(s);
}

int sdd_create_mpu6050(int i2c_bus, int i2c_addr, const char *args) {
    (void)args;

    sdd_mpu6050_state_t *state = calloc(1, sizeof(sdd_mpu6050_state_t));
    if (!state) return -1;

    mpu6050_reset_registers(state);
    state->reg_addr = 0;
    state->phase = 0;

    sdd_device_t dev;
    memset(&dev, 0, sizeof(dev));
    strncpy(dev.name, "mpu6050", SDD_NAME_LEN - 1);
    dev.i2c_bus   = i2c_bus;
    dev.i2c_addr  = i2c_addr;
    dev.i2c_write = mpu6050_i2c_write;
    dev.i2c_read  = mpu6050_i2c_read;
    dev.i2c_start = mpu6050_i2c_start;
    dev.i2c_stop  = mpu6050_i2c_stop;
    dev.spi_bus   = -1;
    dev.cleanup   = mpu6050_cleanup;
    dev.ctx       = state;

    int idx = sdd_register(&dev);
    if (idx < 0) {
        free(state);
        return -1;
    }

    fprintf(stderr, "[SDD] MPU-6050 IMU on I2C%d addr 0x%02X\n", i2c_bus, i2c_addr);
    return idx;
}
