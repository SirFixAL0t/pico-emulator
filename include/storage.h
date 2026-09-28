#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>

/* ========================================================================
 * Flash Write-Through Persistence
 *
 * When a flash file path is set, every flash_range_erase/program operation
 * is immediately synced to disk. This keeps the file always current,
 * allowing external tools to mount and inspect the filesystem at any time.
 *
 * Usage:
 *   ./picoemu firmware.uf2 -flash fs.bin
 *   # In another terminal:
 *   sudo mount -o loop,offset=1048576 fs.bin /mnt/pico
 * ======================================================================== */

/* Set flash persistence file path (enables write-through) */
void flash_persist_set_path(const char *path);

/* Open/create the persistence file (call after firmware load + flash restore) */
int flash_persist_open(void);

/* Sync a region of flash to disk (called by ROM flash intercept) */
void flash_persist_sync(uint32_t offset, uint32_t len);

/* Flush and close persistence file */
void flash_persist_close(void);

/* Full save of entire flash (for exit) */
void flash_persist_save_all(void);

#endif /* STORAGE_H */
