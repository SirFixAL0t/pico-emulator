#!/usr/bin/env python3
"""Live-MP BLE proof: stock Pico-W MicroPython advertises over the vnet room.

Boots ~/mpbuild-stock/firmware.uf2 (has the bluetooth module), drives the
REPL over -stdin/UART0: BLE().active(True) + gap_advertise(), and asserts a
peer on -net-peer sees the ADV announcement (ethertype 0x88B5) carrying the
MP-ADV payload. The emulator announces ADV over vnet on 0x200A enable, so a
foreign scanner observes the MP guest exactly like a bare-metal advertiser.

Usage: python3 test-firmware/mp_ble_room_test.py [mp-uf2]
Exit 0 + MP BLE ROOM PASS on success.
"""
import os
import socket
import struct
import subprocess
import sys
import time

MP_UF2 = sys.argv[1] if len(sys.argv) > 1 else "/home/danish1075/mpbuild-stock/firmware.uf2"
SOCK = "/tmp/mpble_room_test.sock"
EMU = "./build/picoemu"
PAYLOAD = b"MPBT"


def spawn_emu():
    # NOTE: stderr is a SEPARATE pipe from stdout. Merging them into one
    # PIPE (stderr=STDOUT) serializes emulator logging behind guest UART
    # output and the vnet listener bind stalls for many minutes; with
    # split pipes the listener binds in <1s (verified 2026-09-28).
    logf = open("/tmp/mproof/mp_ble_room_emu.log", "wb")
    p = subprocess.Popen(
        [EMU, MP_UF2, "-stdin", "-clock", "125", "-wifi",
         "-net", "-net-peer", SOCK,
         "-timeout", "100", "-max-steps", "900000000"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=logf, bufsize=0)
    p.logf = logf
    # NOTE: the REPL script is written after the peer socket binds (see
    # main); the pipe stays OPEN until the test ends so -stdin never EOFs.
    return p


SCRIPT = ('import bluetooth\r\nb=bluetooth.BLE()\r\nb.active(True)\r\n'
          'print("BLE-ACTIVE")\r\n'
          'b.gap_advertise(100000, b"\\x02\\x01\\x06\\x03\\xffMPBT")\r\n'
          'print("ADV-OK")\r\n')


def recv_frames(s, deadline):
    buf = b""
    while time.time() < deadline:
        try:
            chunk = s.recv(65536)
        except socket.timeout:
            continue
        if not chunk:
            break
        buf += chunk
        while len(buf) >= 4:
            (ln,) = struct.unpack("<I", buf[:4])
            if ln > 9000 or len(buf) < 4 + ln:
                break
            yield buf[4:4 + ln]
            buf = buf[4 + ln:]


def main():
    try:
        os.unlink(SOCK)
    except OSError:
        pass
    emu = spawn_emu()
    # Wait for the vnet listener (bound from the poll loop), THEN feed the
    # REPL script; the pipe stays open until the test ends so -stdin never
    # EOFs early (an early close stalls the guest before vnet_poll binds).
    for _ in range(100):
        if os.path.exists(SOCK) or emu.poll() is not None:
            break
        time.sleep(0.2)
    try:
        emu.stdin.write(SCRIPT.encode())
        emu.stdin.flush()
    except (BrokenPipeError, OSError):
        print("MP BLE ROOM FAIL: emulator exited before REPL script")
        return 1
    try:
        # The emulator LISTENS on SOCK (first starter binds); the test peer
        # CONNECTS, same as ble_peer_test.py / mp6_peer_test.py.
        for _ in range(100):
            try:
                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.settimeout(0.5)
                s.connect(SOCK)
                break
            except OSError:
                if emu.poll() is not None:
                    print("MP BLE ROOM FAIL: emulator exited early")
                    return 1
                time.sleep(0.2)
        else:
            print("MP BLE ROOM FAIL: no peer socket")
            return 1
        with s:
            deadline = time.time() + 95
            for f in recv_frames(s, deadline):
                if len(f) >= 14 + 8 and f[12:14] == b"\x88\xb5" and PAYLOAD in f:
                    # Drain remaining guest UART so the ADV-OK print (which
                    # follows the 0x200A enable by milliseconds of guest
                    # time) is captured before we read.
                    time.sleep(3)
                    emu.stdout._mp_ble_done = True
                    print("MP BLE ROOM PASS (peer saw MP ADV 0x88B5 + MPBT)")
                    out = b""
                    emu.stdout.flush() if hasattr(emu.stdout, "flush") else None
                    import select
                    while select.select([emu.stdout], [], [], 0.5)[0]:
                        chunk = emu.stdout.read1(65536) if hasattr(emu.stdout, "read1") else emu.stdout.read(65536)
                        if not chunk:
                            break
                        out += chunk
                    emu.kill()
                    text = out.decode("utf-8", "replace").replace("\x00", "")
                    for mark in ["BLE-ACTIVE", "ADV-OK"]:
                        print(f"  guest {mark}: {'FOUND' if mark in text else 'MISSING'}")
                    return 0
            print("MP BLE ROOM FAIL: no MP ADV frame (timeout)")
            return 1
    finally:
        if emu.poll() is None:
            try:
                emu.wait(timeout=10)
            except Exception:
                emu.kill()


if __name__ == "__main__":
    sys.exit(main())
