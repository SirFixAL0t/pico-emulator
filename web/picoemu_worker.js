/* Pico-emu WASM worker: runs emulation off main thread.
 * Main thread keeps UI + serial/GPIO; worker steps WASM and posts results.
 * Falls back to main-thread cooperative stepping when SAB unavailable.
 *
 * Protocol (main -> worker): {cmd:'init', arch}, {cmd:'load', data, name},
 *   {cmd:'run', on}, {cmd:'step', n}, {cmd:'uart', ch}, {cmd:'set', k, v}
 * Protocol (worker -> main): {type:'uart', data}, {type:'state', pc, sp, halted, mips},
 *   {type:'gpio', out, oe}, {type:'gdb', data}, {type:'ready'}
 */
let mod = null;
let running = false;
let total = 0;

async function ensureMod() {
  if (mod) return mod;
  const { default: PicoemuModule } = await import('./picoemu.wasm.js');
  mod = await PicoemuModule({ print: () => {}, printErr: () => {} });
  return mod;
}

self.onmessage = async (e) => {
  const m = e.data;
  try {
    if (m.cmd === 'init') {
      await ensureMod();
      mod._picoemu_init(m.arch || 0);
      self.postMessage({ type: 'ready' });
    } else if (m.cmd === 'load') {
      await ensureMod();
      const u8 = new Uint8Array(m.data);
      const ptr = mod._malloc(u8.length);
      mod.HEAPU8.set(u8, ptr);
      let r;
      if ((m.name || '').endsWith('.uf2')) r = mod._picoemu_load_uf2(ptr, u8.length);
      else r = mod._picoemu_load_elf(ptr, u8.length);
      mod._free(ptr);
      mod._picoemu_reset();
      total = 0;
      self.postMessage({ type: 'loaded', blocks: r });
    } else if (m.cmd === 'run') {
      running = !!m.on;
    } else if (m.cmd === 'step') {
      await ensureMod();
      const n = m.n || 2500000;
      const s = mod._picoemu_step(n);
      total += s;
      const out = [];
      let ch;
      while ((ch = mod._picoemu_read_uart(0)) !== -1) out.push(ch);
      // core state
      const pcP = mod._malloc(4), spP = mod._malloc(4);
      mod._picoemu_get_core_state(0, pcP, spP);
      const pc = mod.HEAPU32[pcP >> 2], sp = mod.HEAPU32[spP >> 2];
      mod._free(pcP); mod._free(spP);
      self.postMessage({
        type: 'frame', steps: s, total,
        uart: out, pc, sp,
        halted: mod._picoemu_is_halted(),
        out: mod._picoemu_get_gpio_out ? mod._picoemu_get_gpio_out() : 0,
      });
    } else if (m.cmd === 'uart') {
      await ensureMod();
      mod._picoemu_write_uart(m.ch);
    }
  } catch (err) {
    self.postMessage({ type: 'error', message: String(err && err.message || err) });
  }
};

// auto-run loop when running=true (cooperative inside worker)
setInterval(async () => {
  if (!running || !mod) return;
  try {
    const s = mod._picoemu_step(2500000);
    total += s;
    const out = [];
    let ch, guard = 0;
    while ((ch = mod._picoemu_read_uart(0)) !== -1 && guard++ < 4096) out.push(ch);
    if (out.length || s > 0) self.postMessage({ type: 'frame', steps: s, total, uart: out });
  } catch (e) {}
}, 16);
