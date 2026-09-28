#!/bin/bash
set -e
# pthread variant for true dual-core workers (requires COOP/COEP + SAB).
# Serve via: python3 web/serve_coop.py 8080  (sets Cross-Origin-Isolated)
# Browser must show "yes (SAB)" in Threads panel; else falls back to cooperative.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"
source "$SCRIPT_DIR/../emsdk/emsdk_env.sh" 2>/dev/null || {
  export EMSDK="$SCRIPT_DIR/../emsdk"
  export PATH="$EMSDK:$EMSDK/upstream/emscripten:$PATH"
}
mkdir -p web
COMMON_FLAGS="-O3 -msimd128 -pthread -Wall -Wno-macro-redefined -Wno-logical-not-parentheses -Wno-format"
INCLUDES="-Iinclude/ -Iinclude/rp2350_rv -Iinclude/rp2350_arm"
SOURCES=(
  src/picoemu_wasm.c src/fuse_mount_wasm.c src/wasm_net.c
  src/cpu.c src/instructions.c src/thumb32.c src/membus.c
  src/uf2.c src/elf.c src/gpio.c src/timer.c src/uart.c
  src/spi.c src/i2c.c src/pwm.c src/adc.c src/dma.c
  src/pio.c src/nvic.c src/clocks.c src/usb.c src/rtc.c
  src/rom.c src/gdb.c src/storage.c src/sdcard.c src/emmc.c
  src/fatfs.c src/w5500.c src/bme280.c src/cyw43.c
  src/devtools.c src/vnet.c src/sdd.c src/sdd_thermo.c src/sdd_eeprom.c
  src/rp2350_rv/rv_cpu.c src/rp2350_rv/rv_clint.c
  src/rp2350_rv/rv_membus.c src/rp2350_rv/rv_bootrom.c
  src/rp2350_rv/rp2350_periph.c src/rp2350_rv/picobin.c
  src/rp2350_arm/m33_cpu.c
)
# reuse export list from build_wasm.sh
EXPORTS=$(grep -o '"_[A-Za-z0-9_]*"' build_wasm.sh | paste -sd, -)
EXPORTS="[${EXPORTS}]"
emcc "${SOURCES[@]}" $INCLUDES $COMMON_FLAGS \
  -s WASM=1 -s ALLOW_MEMORY_GROWTH=1 -s MODULARIZE=1 -s EXPORT_NAME="PicoemuModule" \
  -s EXPORTED_FUNCTIONS="$EXPORTS" \
  -s EXPORTED_RUNTIME_METHODS='["ccall","cwrap","getValue","setValue","HEAPU8","HEAPU32","HEAP8"]' \
  -s INITIAL_MEMORY=67108864 -s MAXIMUM_MEMORY=268435456 -s STACK_SIZE=1048576 \
  -s NO_EXIT_RUNTIME=1 -s ENVIRONMENT='web,worker,node' -s EXPORT_ES6=1 \
  -s USE_PTHREADS=1 -s PTHREAD_POOL_SIZE=2 \
  -o web/picoemu.wasm.threads.js
echo "threads build done (serve with serve_coop.py for SAB)"
