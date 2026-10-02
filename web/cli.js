#!/usr/bin/env node
// pico-emu — run RP2040/RP2350 (M0+/M33/RV32) UF2 firmware in Node.
// Usage: pico-emu <firmware.uf2> [--arch auto|m0|m33|rv32] [--clock 125]
//        [--steps 2000000] [--timeout 30] [--cores 2] [--wifi]
//        [--gateway ws://localhost:5090/api/network-gateway] [--room myroom]
//        [--ble-hci ws://localhost:5090/api/ble-gateway]
//        [--board pico-eth|pico-eth2|pico-w6300|pico-w6300-2] [--board-spi 0|1] [--board-live|--board6300-live]
//        [--net-w5500 ws://localhost:8765/w5500]  (live W5500/W6300 proxy pump)
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const mod = await (await import(path.join(HERE, 'picoemu.wasm.js'))).default({
  print: () => {}, printErr: () => {},
});

const args = process.argv.slice(2);
const opt = (name, def) => {
  const i = args.indexOf(name);
  return i >= 0 && i + 1 < args.length ? args[i + 1] : def;
};
const file = args.find((a) => !a.startsWith('--'));
if (!file) {
  console.error('Usage: pico-emu <firmware.uf2> [--arch auto|m0|m33|rv32] [--clock 125] [--steps 2000000] [--timeout 30] [--cores 2] [--wifi] [--gateway URL] [--room ID] [--ble-hci URL] [--board pico-eth|pico-eth2|pico-w6300|pico-w6300-2] [--board-spi 0|1] [--board-live|--board6300-live] [--net-w5500 URL]');
  process.exit(2);
}
const u8 = new Uint8Array(fs.readFileSync(file));
// UF2 family ID lives at offset 28 of block 0
const FAM = { 0xe48bff56: 0, 0xe48bff59: 2, 0xe48bff5a: 1 };
let arch = { auto: -1, m0: 0, rv32: 1, m33: 2 }[opt('--arch', 'auto')] ?? -1;
if (arch < 0) {
  const fam = u8.length >= 32
    ? (u8[28] | (u8[29] << 8) | (u8[30] << 16) | (u8[31] << 24)) >>> 0
    : 0;
  arch = FAM[fam] ?? 0;
}
const clock = parseInt(opt('--clock', '125'), 10);
const budget = parseInt(opt('--steps', '2000000'), 10);
const timeoutS = parseFloat(opt('--timeout', '30'));
const cores = parseInt(opt('--cores', '2'), 10);

mod._picoemu_init(arch);
mod._picoemu_set_clock(clock);
try { mod._picoemu_set_cores(cores); } catch { /* older builds */ }
const ptr = mod._malloc(u8.length);
mod.HEAPU8.set(u8, ptr);
const ok = mod._picoemu_load_uf2(ptr, u8.length);
mod._free(ptr);
if (!ok) { console.error('picoemu: UF2 load failed'); process.exit(1); }
mod._picoemu_reset();

// Optional CYW43 WiFi (--wifi enables the model; with --gateway the
// fake DHCP/DNS server is disabled like native -nodhcp).
{
  const wantWifi = args.includes('--wifi') || opt('--gateway', '') !== '' || opt('--ble-hci', '') !== '';
  if (wantWifi) {
    const nodhcp = opt('--gateway', '') !== '' ? 1 : 0;
    try { mod._picoemu_wifi_enable(nodhcp); } catch {}
  }
  try { if (opt('--ble-hci', '') !== '') mod._picoemu_bt_hci_enable(1); } catch {}
}

// pico-eth/pico-eth2 (W5500) and pico-w6300/pico-w6300-2 (W6300) boards:
// separate SPI boards, off by default. --board-live mirrors SEND to the
// WS proxy like -net-live. Pico2 wiring is identical; only the SoC differs.
// W6300 rides QSPI-single on the same PL022 path (CSn=16/RSTn=22/INTn=15).
{
  const board = opt('--board', '');
  const is6300 = board === 'pico-w6300' || board === 'pico-w6300-2';
  if (board) {
    if (board !== 'pico-eth' && board !== 'pico-eth2' && !is6300) { console.error(`picoemu: unknown board '${board}' (use pico-eth|pico-eth2|pico-w6300|pico-w6300-2)`); process.exit(2); }
    let spi = parseInt(opt('--board-spi', '0'), 10);
    if (!(spi === 0 || spi === 1)) { console.error('picoemu: --board-spi must be 0 or 1'); process.exit(2); }
    const live = (args.includes('--board-live') || args.includes('--board6300-live')) ? 1 : 0;
    try {
      if (is6300) mod._picoemu_board_eth6300(1, live, spi);
      else mod._picoemu_board_eth(1, live, spi);
    } catch {}
    // The proxy protocol is chip-agnostic (same CONNECT/LISTEN/CLOSE/SEND
    // framing), so both boards share the /w5500 path; route RX by board.
    try { opt._isW6300 = is6300 ? 1 : 0; } catch {}
    if (live && !opt('--net-w5500', '') && !(opt('--net-url', '') || opt('--net', ''))) {
      try { opt._autoW5500 = 'ws://localhost:8765/w5500'; } catch {}
    }
  }
}

if (process.stdin.isTTY) process.stdin.setRawMode(true);
process.stdin.resume();
process.stdin.on('data', (d) => {
  for (const b of d) {
    if (b === 3) process.exit(0); // Ctrl-C
    mod._picoemu_write_uart(b);
  }
});

const tick = () => new Promise((r) => setImmediate(r));
const t0 = Date.now();
let done = 0;
const CHUNK = 100000;

// Optional network gateway (raw ETH frames, OpenHW-gateway protocol).
let gw = null;
{
  let url = opt('--gateway', '');
  const room = opt('--room', '');
  if (url) {
    if (room) url += (url.includes('?') ? '&' : '?') + 'sessionId=' + encodeURIComponent(room);
    // Enable the WS uplink mirror IMMEDIATELY (before first step): the
    // guest sends its RS within the first chunk, long before the WS
    // onopen fires. Frames are queued in-WASM and drained once the
    // socket opens, so nothing is lost. onopen re-asserts (idempotent).
    try { mod._picoemu_eth_set_uplink(1); } catch {}
    gw = new WebSocket(url);
    gw.binaryType = 'arraybuffer';
    gw.onopen = () => { try { mod._picoemu_eth_set_uplink(1); } catch {} };
    gw.onclose = () => { try { mod._picoemu_eth_set_uplink(0); } catch {} };
    gw.onmessage = (e) => {
      const arr = e.data instanceof ArrayBuffer ? new Uint8Array(e.data) : new Uint8Array(0);
      if (arr.length < 14 || arr.length > 1522) return;
      const p = mod._malloc(arr.length);
      mod.HEAPU8.set(arr, p);
      try { mod._picoemu_eth_push_rx(p, arr.length); } catch {}
      mod._free(p);
    };
    gw.onerror = (e) => console.error('picoemu: gateway error ' + url + (e && e.message ? ' (' + e.message + ')' : ''));
  }
}
// Optional live-W5500 proxy socket (shared net_proxy /w5500 path; the proxy
// dials real TCP/UDP. NOT the OpenHW gateway: W5500 frames are socket-level
// CONNECT/LISTEN/CLOSE/SEND, not raw ETH).
// Auto-uses the Net socket when it targets /w5500, else --net-w5500 URL.
let w5500ws = null;
{
  let url = opt('--net-w5500', '') || (opt._autoW5500 || '');
  if (!url) {
    const netUrl = opt('--net-url', '') || opt('--net', '');
    if (netUrl && /\/w5500/.test(netUrl)) url = netUrl;
  }
  if (url) {
    w5500ws = new WebSocket(url);
    w5500ws.binaryType = 'arraybuffer';
    w5500ws.onmessage = (e) => {
      const arr = e.data instanceof ArrayBuffer ? new Uint8Array(e.data)
        : typeof e.data === 'string' ? new TextEncoder().encode(e.data) : new Uint8Array(0);
      if (!arr.length) return;
      // Route to the W6300 when a pico-w6300 board is selected (same
      // 8-socket model and proxy framing; chip-agnostic proxy).
      const is6300 = (() => { try { return opt._isW6300 ? 1 : 0; } catch { return 0; } })();
      const pushRx = (sock, p, len) => {
        try {
          if (is6300) mod._picoemu_w6300_push_rx(sock, p, len);
          else mod._picoemu_w5500_push_rx(sock, p, len);
        } catch {}
      };
      const pushStatus = (sock, code) => {
        try {
          if (is6300) mod._picoemu_w6300_push_status(sock, code);
          else mod._picoemu_w5500_push_status(sock, code);
        } catch {}
      };
      if (arr[0] === 0x53 && arr.length >= 4) {
        pushStatus(arr[1], arr[3]);
      } else if (arr[0] < 8 && arr.length >= 3) {
        const ln = arr[1] | (arr[2] << 8);
        const payload = arr.slice(3, 3 + ln);
        const p = mod._malloc(payload.length);
        mod.HEAPU8.set(payload, p);
        pushRx(arr[0], p, payload.length);
        mod._free(p);
      } else if (arr[0] === 0x57 && arr.length >= 4) {
        const sock = arr[1], ln = arr[2] | (arr[3] << 8);
        const payload = arr.slice(4, 4 + ln);
        const p = mod._malloc(payload.length);
        mod.HEAPU8.set(payload, p);
        pushRx(sock, p, payload.length);
        mod._free(p);
      }
    };
    w5500ws.onerror = (e) => console.error('picoemu: net-w5500 error ' + url + (e && e.message ? ' (' + e.message + ')' : ''));
  }
}
// Optional BLE HCI uplink (raw H4 packets, Bumble/gateway ble-gateway protocol).
let blehci = null;
{
  const url = opt('--ble-hci', '');
  if (url) {
    blehci = new WebSocket(url);
    blehci.binaryType = 'arraybuffer';
    blehci.onmessage = (e) => {
      const arr = e.data instanceof ArrayBuffer ? new Uint8Array(e.data) : new Uint8Array(0);
      if (arr.length < 2 || arr.length > 1088) return;
      const p = mod._malloc(arr.length);
      mod.HEAPU8.set(arr, p);
      try { mod._picoemu_bt_hci_push_rx(p, arr.length); } catch {}
      mod._free(p);
    };
    blehci.onerror = (e) => console.error('picoemu: ble-hci error ' + url + (e && e.message ? ' (' + e.message + ')' : ''));
  }
}
process.stdout.write('');
while (done < budget && (Date.now() - t0) / 1000 < timeoutS) {
  mod._picoemu_step(Math.min(CHUNK, budget - done));
  done += CHUNK;
  await tick(); // let stdin/stdio/events fire
  if (gw && gw.readyState === 1) {
    for (let i = 0; i < 16; i++) {
      const p = mod._malloc(2048);
      let got = -1;
      try { got = mod._picoemu_eth_pop_tx(p, 2048); } catch { mod._free(p); break; }
      if (got <= 0) { mod._free(p); break; }
      try { gw.send(mod.HEAPU8.slice(p, p + got)); } catch {}
      mod._free(p);
    }
  }
  if (blehci && blehci.readyState === 1) {
    for (let i = 0; i < 16; i++) {
      const p = mod._malloc(2048);
      let got = -1;
      try { got = mod._picoemu_bt_hci_pop_tx(p, 2048); } catch { mod._free(p); break; }
      if (got <= 0) { mod._free(p); break; }
      try { blehci.send(mod.HEAPU8.slice(p, p + got)); } catch {}
      mod._free(p);
    }
  }
  // W5500/W6300 proxy pump (Node): forward queued CONNECT/LISTEN/CLOSE/SEND
  // to the --net-w5500 proxy socket. Same shared net_proxy as the browser
  // Net panel; NOT the OpenHW gateway (socket-level TCP/UDP, not ETH).
  // Both chips queue identical framing; pump each queue in turn.
  if (w5500ws && w5500ws.readyState === 1) {
    const pumpOne = (lenFn, popFn) => {
      let budget = 0;
      try { budget = mod[lenFn](); } catch { budget = 0; }
      if (budget > 0) {
        const n = Math.min(budget, 8192);
        const p = mod._malloc(n);
        let got = 0;
        try { got = mod[popFn](p, n); } catch { got = 0; }
        if (got > 0) { try { w5500ws.send(mod.HEAPU8.slice(p, p + got)); } catch {} }
        mod._free(p);
      }
    };
    try { pumpOne('_picoemu_w5500_tx_len', '_picoemu_w5500_pop_tx'); } catch {}
    try { pumpOne('_picoemu_w6300_tx_len', '_picoemu_w6300_pop_tx'); } catch {}
  }
  let s = '', ch, n = 0;
  while ((ch = mod._picoemu_read_uart(0)) !== -1 && n++ < 65536) s += String.fromCharCode(ch);
  if (s) process.stdout.write(s);
  try { if (mod._picoemu_is_halted()) break; } catch { /* ignore */ }
}
process.exit(0);
