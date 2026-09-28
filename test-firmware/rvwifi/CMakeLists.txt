# RV32 WiFi guests (RP2350 Hazard3, pico-sdk lwip_poll), vendored from
# ~/rvwifi 2026-09-28. Build:
#   cmake -S test-firmware/rvwifi -B /tmp/rvwifi -DPICO_SDK_PATH=<sdk>
#   cmake --build /tmp/rvwifi
# Output rvwifi_join.uf2 -> web/wifi_join_rv32.uf2 (JOIN_SSID Pico-emuNet).
cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2_w)
set(PICO_PLATFORM rp2350-riscv)
set(PICO_TOOLCHAIN pico_riscv_gcc)
if(NOT DEFINED PICO_SDK_PATH)
  set(PICO_SDK_PATH "$ENV{HOME}/.arduino15/packages/rp2040/hardware/rp2040/6.0.0/pico-sdk")
endif()
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
project(rvwifi C CXX ASM)
pico_sdk_init()
add_executable(rvwifi_scan picow_wifi_scan.c)
target_link_libraries(rvwifi_scan pico_stdlib pico_cyw43_arch_none)
pico_add_extra_outputs(rvwifi_scan)
add_executable(rvwifi_join picow_wifi_join.c)
target_include_directories(rvwifi_join PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(rvwifi_join pico_stdlib pico_cyw43_arch_lwip_poll)
pico_add_extra_outputs(rvwifi_join)
