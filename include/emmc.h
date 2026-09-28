#ifndef EMMC_H
#define EMMC_H

#include <stdint.h>
#include <stddef.h>

/* ========================================================================
 * eMMC SPI-Mode Emulation
 *
 * Emulates an eMMC device using SPI bus protocol. Similar to SD card
 * but with eMMC-specific initialization (CMD1 instead of ACMD41),
 * different CSD/CID, and extended command support.
 *
 * Backed by a file on the host filesystem for persistent storage.
 * Attaches to an SPI bus via spi_attach_device().
 *
 * Supported commands:
 *   CMD0  (GO_IDLE_STATE)      - Reset device
 *   CMD1  (SEND_OP_COND)       - Initialize (eMMC-specific)
 *   CMD8  (SEND_EXT_CSD)       - Extended CSD (eMMC)
 *   CMD9  (SEND_CSD)           - Card-specific data
 *   CMD10 (SEND_CID)           - Card identification
 *   CMD12 (STOP_TRANSMISSION)  - Stop multi-block read
 *   CMD13 (SEND_STATUS)        - Status
 *   CMD16 (SET_BLOCKLEN)       - Set block length
 *   CMD17 (READ_SINGLE_BLOCK)  - Read one block
 *   CMD18 (READ_MULTIPLE_BLOCK) - Read multiple blocks
 *   CMD24 (WRITE_BLOCK)        - Write one block
 *   CMD25 (WRITE_MULTIPLE_BLOCK) - Write multiple blocks
 *   CMD58 (READ_OCR)           - Read OCR register
 *
 * Usage:
 *   ./picoemu firmware.uf2 -emmc emmc.img
 *   ./picoemu firmware.uf2 -emmc emmc.img -emmc-size 128
 *   ./picoemu firmware.uf2 -emmc emmc.img -emmc-spi 1
 * ======================================================================== */

#define EMMC_BLOCK_SIZE     512
#define EMMC_DEFAULT_SIZE   (128 * 1024 * 1024)  /* 128 MB default */

/* eMMC states */
typedef enum {
    EMMC_STATE_IDLE,
    EMMC_STATE_READY,
    EMMC_STATE_SENDING,
    EMMC_STATE_RECEIVING,
    EMMC_STATE_SEND_MULTI,
    EMMC_STATE_RECV_MULTI,
} emmc_state_t;

typedef struct {
    /* Device state */
    emmc_state_t state;
    int initialized;
    int cs_active;

    /* Backing storage */
    uint8_t *data;
    size_t size;
    char path[256];
    int dirty;

    /* Command assembly */
    uint8_t cmd_buf[6];
    int cmd_pos;

    /* Response queue */
    uint8_t resp[EMMC_BLOCK_SIZE + 16];
    int resp_len;
    int resp_pos;

    /* Multi-block state */
    uint32_t multi_addr;

    /* Write state */
    uint8_t wr_buf[EMMC_BLOCK_SIZE + 4];
    int wr_pos;
    int wr_expected;
    uint32_t wr_addr;
} emmc_t;

/* Initialize eMMC with backing file */
int emmc_init(emmc_t *em, const char *path, size_t size);

/* Cleanup and flush */
void emmc_cleanup(emmc_t *em);

/* Flush dirty data to backing file */
void emmc_flush(emmc_t *em);

/* SPI transfer callback (for spi_attach_device) */
uint8_t emmc_spi_xfer(void *ctx, uint8_t mosi);

/* SPI chip-select callback */
void emmc_spi_cs(void *ctx, int cs_active);

#endif /* EMMC_H */
