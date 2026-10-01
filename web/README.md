# picoemu

RP2040 / RP2350 (Cortex-M0+, Cortex-M33, RISC-V Hazard3) emulator that runs
in Node or the browser via WebAssembly. Boots real UF2 firmware —
littleOS, MicroPython, and 40+ bare-metal peripheral demos included.

Credit: emulation core is a WASM port of
[Night-Traders-Dev/Bramble](https://github.com/Night-Traders-Dev/Bramble)
(MIT); WASM build, browser UI and packaging by
[danish9661/pico-emulator](https://github.com/danish9661/pico-emulator).

## Install

```sh
npm i pico-emu
```

## CLI — `npx pico-emu firmware.uf2`

```sh
npx pico-emu hello_world.uf2                    # arch auto-detected from UF2
npx pico-emu ./web/uart_echo_rv32.uf2           # type a line, Enter submits
npx pico-emu fw.uf2 --arch rv32 --clock 125 --steps 2000000 --timeout 30 --cores 2
npx pico-emu wifi_fw.uf2 --gateway ws://localhost:5099/api/network-gateway --room lab
```

Type into the terminal to send UART bytes (Ctrl-C quits); firmware output
streams to stdout. `--arch` accepts `auto` (default, UF2 family ID),
`m0`, `m33`, `rv32`.

## JS API

```js
import createEmu from 'pico-emu';
import fs from 'fs';

const mod = await createEmu({ print: () => {}, printErr: () => {} });
mod._picoemu_init(1);            // 0=M0+ 1=RV32 2=M33
mod._picoemu_set_clock(125);     // MHz
const uf2 = new Uint8Array(fs.readFileSync('hello_rv32.uf2'));
const ptr = mod._malloc(uf2.length);
mod.HEAPU8.set(uf2, ptr);
mod._picoemu_load_uf2(ptr, uf2.length);
mod._free(ptr);
mod._picoemu_reset();
mod._picoemu_step(200000);       // run N instructions

let ch, out = '';
while ((ch = mod._picoemu_read_uart(0)) !== -1) out += String.fromCharCode(ch);
console.log(out);                // Hello from Pico-emu RV32!
mod._picoemu_write_uart(65);     // send 'A' to firmware
```

More entry points: `_picoemu_load_elf`, `_picoemu_get_gpio` /
`_picoemu_set_gpio`, `_picoemu_mem_read32` / `_picoemu_mem_write32`,
`_picoemu_set_cores`, `_picoemu_is_halted`. Full reference with every
export: [docs/PICOEMU.md](https://github.com/danish9661/pico-emulator/blob/main/docs/PICOEMU.md).

## Browser

`index.html` is a ready-made UI (serial monitor, GPIO viewer, three demo
dropdowns: RP2040 / M33 / RV32). Serve the package dir and open it, or copy
`picoemu.wasm.*` + `index.html` into your app.

## Firmware in this package

`firmware/` ships a curated demo set (browser UI dropdowns load from here):
`hello_world`, `gpio_test`, `eth_dhcp` + `eth_http` (W5500, pico-eth),
`eth_dhcp6300` + `eth_http6300` + `ethdhcp6300_arduino` (W6300, pico-w6300),
`micropython_rp2040` + `micropython_rp2350`, `ble_adv`, `wifi_scan`,
`littleos`. Full per-arch build output (90 UF2s incl. `_pico2`/`_rv32`
variants) lives in the GitHub repo under `web/` — run the sweep there.

Networking extras: `eth_dhcp{,_pico2,_rv32}.uf2` + `eth_http{,_pico2,_rv32}.uf2`
(W5500 DHCP + HTTP client, pico-eth on SPI0 — pick the board in the UI,
then bridge to the Go gateway for a real lease; offline sweep asserts
`ETH MACRAW-OK`), `eth_dhcp6300{,_pico2,_rv32}.uf2` +
`eth_http6300{,_pico2,_rv32}.uf2` (W6300 DHCP + HTTP client, pico-w6300 on
SPI0 QSPI-single — same gateway, same markers) +
`ethdhcp6300_arduino{,_pico2}.uf2` (real Arduino `W6300lwIP` DHCP, PIO-QSPI
driver runs unmodified; offline sweep asserts `ETH-BEGIN-OK`),
`ble_adv{,_pico2}.uf2` (BLE advertise M0+/M33 over the
CYW43 BT bus — sweep-locked `ARM BLE LISTEN` with `-wifi`, no peer),
`wifi_scan` / `wifi_ping` / `wifi_webserver` + `_pico2` / `_rv32`
(Arduino-CLI WiFi builds).

W6300 feature surface (dual IPv4/IPv6 TCP/IP offload, all modeled):
socket modes TCP4/UDP4/IPRAW4/MACRAW + TCP6/UDP6/IPRAW6 + dual-stack
TCPD/UDPD; commands OPEN/LISTEN/CONNECT/CONNECT6/DISCON/CLOSE/SEND/
SEND_MAC/SEND_KEEP/RECV/SEND6; IPv6 net registers (LLAR/GUAR/SUB6R/GA6R,
SLDIP6R, UIP6R/UPORT6R); masked interrupt chain (Sn_IMR/SIMR/SLIMR/IMR +
IEN gate); RTR/RCR retry engine with TIMEOUT; Sn_KPALVTR auto-keepalive;
NETMR Wake-on-LAN magic-packet detect; Sn_TTLR/Sn_TOSR/Sn_MSSR socket
options; live host sockets for IPv4 + IPv6 (loopback-verified).

## License

MIT — see LICENSE (© 2026 danish9661; upstream © 2025 Night-Traders-Dev).
