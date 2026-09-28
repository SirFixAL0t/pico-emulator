#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "emulator.h"
#include "fuse_mount.h"
#include <emscripten.h>

static uint8_t *wasm_flash = NULL;
static size_t wasm_flash_size = 0;
static int wasm_mounted = 0;
static uint32_t wasm_flash_offset = 0;

int fuse_mount_start(uint8_t *flash_data, size_t flash_size, const char *mount_point) {
    wasm_flash = flash_data; (void)wasm_flash;
    wasm_flash_size = flash_size; (void)wasm_flash_size;
    wasm_mounted = 1;
    EM_ASM({
        if (typeof navigator !== 'undefined' && navigator.storage && navigator.storage.getDirectory) {
            console.log("[WASM-FUSE] OPFS available for", UTF8ToString($0));
        } else {
            console.log("[WASM-FUSE] MEMFS fallback for", UTF8ToString($0));
        }
        /* Persist mount descriptor so UI can show it */
        if (typeof window !== 'undefined') {
            window.picoemuMountPoint = UTF8ToString($0);
        }
    }, mount_point);
    /* Persist current flash image to MEMFS so it survives resets, and try
     * IDBFS/OPFS sync if the FS is mounted by the shell. */
    EM_ASM({
        try {
            if (typeof FS !== 'undefined') {
                try { FS.mkdir('/persist'); } catch(e) {}
                try { FS.mount(IDBFS, {}, '/persist'); } catch(e) {}
            }
        } catch(e) {}
    });
    FILE *f = fopen("/persist/picoemu_flash.bin", "wb");
    if (f) {
        fwrite(flash_data, 1, flash_size, f);
        fclose(f);
        EM_ASM({
            try {
                if (typeof FS !== 'undefined' && FS.syncfs) {
                    FS.syncfs(function(err) {
                        if (err) console.log("[WASM-FUSE] syncfs err", err);
                        else console.log("[WASM-FUSE] flash persisted to IDBFS");
                    });
                }
            } catch(e) {}
        });
    } else {
        /* MEMFS fallback */
        FILE *m = fopen("/flash.bin", "wb");
        if (m) { fwrite(flash_data, 1, flash_size, m); fclose(m); }
    }
    fprintf(stderr, "[WASM-FUSE] Mounted %zu bytes at %s (OPFS/MEMFS, off=0x%08X)\n",
            flash_size, mount_point, wasm_flash_offset);
    return 0;
}
void fuse_mount_stop(void) { wasm_mounted = 0; }
int fuse_mount_active(void) { return wasm_mounted; }
void fuse_set_flash_offset(uint32_t offset) { wasm_flash_offset = offset; }
uint32_t fuse_get_flash_offset(void) { return wasm_flash_offset; }
pthread_mutex_t fuse_flash_mutex = PTHREAD_MUTEX_INITIALIZER;
