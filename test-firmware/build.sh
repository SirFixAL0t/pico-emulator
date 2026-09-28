#!/bin/bash

set -e

echo "Building Pico-emu test firmware..."

mkdir -p build

cd build

# Determine which firmware to build
TARGET="${1:-hello_world}"

case "$TARGET" in

hello_world|hello)

echo "[1/3] Compiling hello_world.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../hello_world.S -o hello_world.o

echo "[2/3] Linking..."
arm-none-eabi-ld -T ../linker.ld hello_world.o -o hello_world.elf

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary hello_world.elf hello_world.bin

python3 ../uf2conv.py hello_world.bin -o ../../hello_world.uf2 -b 0x10000100 -f 0xE48BFF56

echo "✓ Build complete: hello_world.uf2"

;;

gpio|gpio_test)

echo "[1/3] Compiling gpio_test.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../gpio_test.S -o gpio_test.o

echo "[2/3] Linking..."
arm-none-eabi-ld -T ../linker.ld gpio_test.o -o gpio_test.elf

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary gpio_test.elf gpio_test.bin

python3 ../uf2conv.py gpio_test.bin -o ../../gpio_test.uf2 -b 0x10000100 -f 0xE48BFF56

echo "✓ Build complete: gpio_test.uf2"

;;

timer|timer_test)

echo "[1/3] Compiling timer_test.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../timer_test.S -o timer_test.o

echo "[2/3] Linking..."
arm-none-eabi-ld -T ../linker.ld timer_test.o -o timer_test.elf

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary timer_test.elf timer_test.bin

python3 ../uf2conv.py timer_test.bin -o ../../timer_test.uf2 -b 0x10000100 -f 0xE48BFF56

echo "✓ Build complete: timer_test.uf2"

;;

interrupt|interrupt_test)

echo "[1/3] Compiling interrupt_test.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../interrupt_test.S -o interrupt_test.o

echo "[2/3] Linking..."
arm-none-eabi-ld -T ../linker.ld interrupt_test.o -o interrupt_test.elf

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary interrupt_test.elf interrupt_test.bin

python3 ../uf2conv.py interrupt_test.bin -o ../../interrupt_test.uf2 -b 0x10000100 -f 0xE48BFF56

echo "✓ Build complete: interrupt_test.uf2"

;;

clocks|clocks_test)

echo "[1/3] Compiling clocks_test.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../clocks_test.S -o clocks_test.o

echo "[2/3] Linking..."
arm-none-eabi-ld -T ../linker.ld clocks_test.o -o clocks_test.elf

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary clocks_test.elf clocks_test.bin

python3 ../uf2conv.py clocks_test.bin -o ../../clocks_test.uf2 -b 0x10000100 -f 0xE48BFF56

echo "✓ Build complete: clocks_test.uf2"

;;

psm|psm_test)

echo "[1/3] Compiling psm_test.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../psm_test.S -o psm_test.o

echo "[2/3] Linking..."
arm-none-eabi-ld -T ../linker.ld psm_test.o -o psm_test.elf

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary psm_test.elf psm_test.bin

python3 ../uf2conv.py psm_test.bin -o ../../psm_test.uf2 -b 0x10000100 -f 0xE48BFF56

echo "✓ Build complete: psm_test.uf2"

;;

fp|fp_test)

echo "[1/3] Compiling fp_test.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../fp_test.S -o fp_test.o

echo "[2/3] Linking (libgcc for softfloat __aeabi_dadd)..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -nostartfiles -T ../linker.ld fp_test.o -lgcc -o fp_test.elf

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary fp_test.elf fp_test.bin

python3 ../uf2conv.py fp_test.bin -o ../../fp_test.uf2 -b 0x10000100 -f 0xE48BFF56

echo "✓ Build complete: fp_test.uf2"

;;

ws2812|ws2812_test)

echo "[1/3] Compiling ws2812_test.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../ws2812_test.S -o ws2812_test.o

echo "[2/3] Linking..."
arm-none-eabi-ld -T ../linker.ld ws2812_test.o -o ws2812_test.elf

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary ws2812_test.elf ws2812_test.bin

python3 ../uf2conv.py ws2812_test.bin -o ../../ws2812_test.uf2 -b 0x10000100 -f 0xE48BFF56

echo "✓ Build complete: ws2812_test.uf2"

;;

rtc|rtc_test)

echo "[1/3] Compiling rtc_test.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../rtc_test.S -o rtc_test.o

echo "[2/3] Linking..."
arm-none-eabi-ld -T ../linker.ld rtc_test.o -o rtc_test.elf

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary rtc_test.elf rtc_test.bin

python3 ../uf2conv.py rtc_test.bin -o ../../rtc_test.uf2 -b 0x10000100 -f 0xE48BFF56

echo "✓ Build complete: rtc_test.uf2"

;;

uart_echo|uart-echo|echo)

echo "[1/3] Compiling uart_echo.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../uart_echo.S -o uart_echo.o

echo "[2/3] Linking..."
arm-none-eabi-ld -T ../linker.ld uart_echo.o -o uart_echo.elf

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary uart_echo.elf uart_echo.bin

python3 ../uf2conv.py uart_echo.bin -o ../../uart_echo.uf2 -b 0x10000100 -f 0xE48BFF56

echo "✓ Build complete: uart_echo.uf2"

;;

name_prompt|prompt|name)

echo "[1/3] Compiling name_prompt.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../name_prompt.S -o name_prompt.o

echo "[2/3] Linking..."
arm-none-eabi-ld -T ../linker.ld name_prompt.o -o name_prompt.elf

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary name_prompt.elf name_prompt.bin

python3 ../uf2conv.py name_prompt.bin -o ../../name_prompt.uf2 -b 0x10000100 -f 0xE48BFF56

echo "✓ Build complete: name_prompt.uf2"

;;

all)

echo "Building all test firmware..."

# Hello World
echo " - Building hello_world.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../hello_world.S -o hello_world.o
arm-none-eabi-ld -T ../linker.ld hello_world.o -o hello_world.elf
arm-none-eabi-objcopy -O binary hello_world.elf hello_world.bin
python3 ../uf2conv.py hello_world.bin -o ../../hello_world.uf2 -b 0x10000100 -f 0xE48BFF56
echo " ✓ hello_world.uf2"

# GPIO Test
echo " - Building gpio_test.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../gpio_test.S -o gpio_test.o
arm-none-eabi-ld -T ../linker.ld gpio_test.o -o gpio_test.elf
arm-none-eabi-objcopy -O binary gpio_test.elf gpio_test.bin
python3 ../uf2conv.py gpio_test.bin -o ../../gpio_test.uf2 -b 0x10000100 -f 0xE48BFF56
echo " ✓ gpio_test.uf2"

# Timer Test
echo " - Building timer_test.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../timer_test.S -o timer_test.o
arm-none-eabi-ld -T ../linker.ld timer_test.o -o timer_test.elf
arm-none-eabi-objcopy -O binary timer_test.elf timer_test.bin
python3 ../uf2conv.py timer_test.bin -o ../../timer_test.uf2 -b 0x10000100 -f 0xE48BFF56
echo " ✓ timer_test.uf2"

# Interrupt Test
echo " - Building interrupt_test.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../interrupt_test.S -o interrupt_test.o
arm-none-eabi-ld -T ../linker.ld interrupt_test.o -o interrupt_test.elf
arm-none-eabi-objcopy -O binary interrupt_test.elf interrupt_test.bin
python3 ../uf2conv.py interrupt_test.bin -o ../../interrupt_test.uf2 -b 0x10000100 -f 0xE48BFF56
echo " ✓ interrupt_test.uf2"

# Interactive UART prompt test
echo " - Building name_prompt.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../name_prompt.S -o name_prompt.o
arm-none-eabi-ld -T ../linker.ld name_prompt.o -o name_prompt.elf
arm-none-eabi-objcopy -O binary name_prompt.elf name_prompt.bin
python3 ../uf2conv.py name_prompt.bin -o ../../name_prompt.uf2 -b 0x10000100 -f 0xE48BFF56
echo " ✓ name_prompt.uf2"

# Clocks register readout test
echo " - Building clocks_test.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../clocks_test.S -o clocks_test.o
arm-none-eabi-ld -T ../linker.ld clocks_test.o -o clocks_test.elf
arm-none-eabi-objcopy -O binary clocks_test.elf clocks_test.bin
python3 ../uf2conv.py clocks_test.bin -o ../../clocks_test.uf2 -b 0x10000100 -f 0xE48BFF56
echo " ✓ clocks_test.uf2"

# PSM register readout test
echo " - Building psm_test.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../psm_test.S -o psm_test.o
arm-none-eabi-ld -T ../linker.ld psm_test.o -o psm_test.elf
arm-none-eabi-objcopy -O binary psm_test.elf psm_test.bin
python3 ../uf2conv.py psm_test.bin -o ../../psm_test.uf2 -b 0x10000100 -f 0xE48BFF56
echo " ✓ psm_test.uf2"

# Softfloat double test (0.1 + 0.2 via libgcc __aeabi_dadd)
echo " - Building fp_test.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../fp_test.S -o fp_test.o
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -nostartfiles -T ../linker.ld fp_test.o -lgcc -o fp_test.elf
arm-none-eabi-objcopy -O binary fp_test.elf fp_test.bin
python3 ../uf2conv.py fp_test.bin -o ../../fp_test.uf2 -b 0x10000100 -f 0xE48BFF56
echo " ✓ fp_test.uf2"

# WS2812 PIO bitstream test
echo " - Building ws2812_test.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../ws2812_test.S -o ws2812_test.o
arm-none-eabi-ld -T ../linker.ld ws2812_test.o -o ws2812_test.elf
arm-none-eabi-objcopy -O binary ws2812_test.elf ws2812_test.bin
python3 ../uf2conv.py ws2812_test.bin -o ../../ws2812_test.uf2 -b 0x10000100 -f 0xE48BFF56
echo " ✓ ws2812_test.uf2"

# RTC readout test
echo " - Building rtc_test.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../rtc_test.S -o rtc_test.o
arm-none-eabi-ld -T ../linker.ld rtc_test.o -o rtc_test.elf
arm-none-eabi-objcopy -O binary rtc_test.elf rtc_test.bin
python3 ../uf2conv.py rtc_test.bin -o ../../rtc_test.uf2 -b 0x10000100 -f 0xE48BFF56
echo " ✓ rtc_test.uf2"

# UART echo test
echo " - Building uart_echo.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../uart_echo.S -o uart_echo.o
arm-none-eabi-ld -T ../linker.ld uart_echo.o -o uart_echo.elf
arm-none-eabi-objcopy -O binary uart_echo.elf uart_echo.bin
python3 ../uf2conv.py uart_echo.bin -o ../../uart_echo.uf2 -b 0x10000100 -f 0xE48BFF56
echo " ✓ uart_echo.uf2"

# RP2350 M33 variants
for t in clocks_test psm_test ws2812_test rtc_test uart_echo; do
echo " - Building ${t}_pico2.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -DRP2350 -c ../$t.S -o ${t}_pico2.o
arm-none-eabi-ld -T ../linker.ld ${t}_pico2.o -o ${t}_pico2.elf
arm-none-eabi-objcopy -O binary ${t}_pico2.elf ${t}_pico2.bin
python3 ../uf2conv.py ${t}_pico2.bin -o ../../${t}_pico2.uf2 -b 0x10000100 -f 0xE48BFF59
echo " ✓ ${t}_pico2.uf2"
done
echo " - Building fp_test_pico2.uf2..."
arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -DRP2350 -c ../fp_test.S -o fp_test_pico2.o
arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -mfloat-abi=soft -DRP2350 -nostartfiles -T ../linker.ld fp_test_pico2.o -lgcc -o fp_test_pico2.elf
arm-none-eabi-objcopy -O binary fp_test_pico2.elf fp_test_pico2.bin
python3 ../uf2conv.py fp_test_pico2.bin -o ../../fp_test_pico2.uf2 -b 0x10000100 -f 0xE48BFF59
echo " ✓ fp_test_pico2.uf2"

echo ""

echo "✓ All firmware built successfully (17/17)"

;;

*_pico2)

BASE="${1%_pico2}"
echo "[1/3] Compiling $BASE.S for RP2350 M33..."
arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -DRP2350 -c ../$BASE.S -o ${BASE}_pico2.o

echo "[2/3] Linking..."
if [ "$BASE" = "fp_test" ]; then
  arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -mfloat-abi=soft -DRP2350 -nostartfiles -T ../linker.ld ${BASE}_pico2.o -lgcc -o ${BASE}_pico2.elf
else
  arm-none-eabi-ld -T ../linker.ld ${BASE}_pico2.o -o ${BASE}_pico2.elf
fi

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary ${BASE}_pico2.elf ${BASE}_pico2.bin

python3 ../uf2conv.py ${BASE}_pico2.bin -o ../../${BASE}_pico2.uf2 -b 0x10000100 -f 0xE48BFF59

echo "✓ Build complete: ${BASE}_pico2.uf2"

;;

*)

echo "Usage: ./build.sh [TARGET]"
echo ""
echo "Available targets:"
echo " hello_world (default) - Build hello world test"
echo " gpio - Build GPIO test"
echo " timer - Build timer test"
echo " interrupt - Build timer interrupt test (full flow)"
echo " name_prompt - Build interactive UART stdin test"
echo " clocks - Build clocks register readout test"
echo " psm - Build PSM register readout test"
echo " fp - Build softfloat double (0.1+0.2) test"
echo " ws2812 - Build PIO WS2812 bitstream test"
echo " rtc - Build RTC readout test"
echo " uart_echo - Build UART echo test"
echo " <name>_pico2 - RP2350 M33 variant of any target (-DRP2350)"
echo " ws2812 - Build PIO WS2812 bitstream test"
echo " rtc - Build RTC readout test"
echo " uart_echo - Build UART echo test"
echo " all - Build all tests"
echo ""

cd .. && rm -rf build

exit 1

;;

esac

cd .. && rm -rf build

echo "Done!"
