#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include <stddef.h>

/* ========================================================================
 * SD Card SPI-Mode Emulation
 *
 * Emulates a standard SD/SDHC card using the SPI bus protocol.
 * Backed by a file on the host filesystem for persistent storage.
 * Attaches to an SPI bus via spi_attach_device().
 *
 * Supported commands:
 *   CMD0  (GO_IDLE_STATE)      - Reset card
 *   CMD8  (SEND_IF_COND)       - Voltage check (SD v2)
 *   CMD9  (SEND_CSD)           - Card-specific data
 *   CMD10 (SEND_CID)           - Card identification
 *   CMD12 (STOP_TRANSMISSION)  - Stop multi-block read
 *   CMD13 (SEND_STATUS)        - Card status
 *   CMD16 (SET_BLOCKLEN)       - Set block length
 *   CMD17 (READ_SINGLE_BLOCK)  - Read one block
 *   CMD18 (READ_MULTIPLE_BLOCK) - Read multiple blocks
 *   CMD24 (WRITE_BLOCK)        - Write one block
 *   CMD25 (WRITE_MULTIPLE_BLOCK) - Write multiple blocks
 *   CMD55 (APP_CMD)            - Prefix for ACMD
 *   CMD58 (READ_OCR)           - Read OCR register
 *   ACMD41 (SD_SEND_OP_COND)  - Initialize card
 *
 * Usage:
 *   ./picoemu firmware.uf2 -sdcard sd.img
 *   ./picoemu firmware.uf2 -sdcard sd.img -sdcard-spi 0
 * ======================================================================== */

#define SD_BLOCK_SIZE       512
#define SD_DEFAULT_SIZE     (64 * 1024 * 1024)  /* 64 MB default */

/* SD card states */
typedef enum {
    SD_STATE_IDLE,          /* After CMD0, awaiting initialization */
    SD_STATE_READY,         /* Initialized, accepting commands */
    SD_STATE_SENDING,       /* Sending read response data */
    SD_STATE_RECEIVING,     /* Receiving write data from host */
    SD_STATE_SEND_MULTI,    /* Multi-block read in progress */
    SD_STATE_RECV_MULTI,    /* Multi-block write in progress */
} sdcard_state_t;

typedef struct {
    /* Card state */
    sdcard_state_t state;
    int initialized;        /* Card has completed initialization */
    int app_cmd;            /* Next command is ACMD (CMD55 prefix) */
    int sdhc;               /* SDHC mode (block addressing) */
    int cs_active;          /* Chip select asserted */

    /* Backing storage */
    uint8_t *data;          /* Card data buffer */
    size_t size;            /* Total card size in bytes */
    char path[256];         /* Backing file path */
    int dirty;              /* Data modified, needs flush */

    /* Command assembly */
    uint8_t cmd_buf[6];     /* 6-byte command frame */
    int cmd_pos;            /* Bytes collected so far */

    /* Response queue */
    uint8_t resp[SD_BLOCK_SIZE + 16];
    int resp_len;
    int resp_pos;

    /* Multi-block state */
    uint32_t multi_addr;    /* Current block address for multi-block ops */

    /* Write state */
    uint8_t wr_buf[SD_BLOCK_SIZE + 4]; /* token + data + CRC */
    int wr_pos;
    int wr_expected;
    uint32_t wr_addr;      /* Write destination (byte address) */
} sdcard_t;

/* Initialize SD card with backing file (creates if needed) */
int sdcard_init(sdcard_t *sd, const char *path, size_t size);

/* Cleanup and flush to file */
void sdcard_cleanup(sdcard_t *sd);

/* Flush dirty data to backing file */
void sdcard_flush(sdcard_t *sd);

/* SPI transfer callback (for spi_attach_device) */
uint8_t sdcard_spi_xfer(void *ctx, uint8_t mosi);

/* SPI chip-select callback */
void sdcard_spi_cs(void *ctx, int cs_active);

#endif /* SDCARD_H */
