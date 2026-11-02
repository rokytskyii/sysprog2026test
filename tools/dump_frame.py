#!/usr/bin/env python3
"""
Зчитування grayscale-кадру з журналу Incident Recorder.

Варіант A — через UART (авто):
  # Закрийте PuTTY!
  python3 tools/dump_frame.py --port /dev/ttyACM0 --slot 0 --out docs/metrics/frame0 -v

Варіант B — якщо UART з Python «німий» (надійніше на Nucleo):
  1) У PuTTY виконайте: dump frame 0
  2) Скопіюйте ВЕСЬ вивід від FRAME_BEGIN до FRAME_END у файл frame.txt
  3) python3 tools/dump_frame.py --from-log frame.txt --out docs/metrics/frame0
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

BEGIN_RE = re.compile(
    r"FRAME_BEGIN slot=(\d+) w=(\d+) h=(\d+) bytes=(\d+)"
)
END_RE = re.compile(r"FRAME_END ok=(\d+)(?: payload_crc=([0-9A-Fa-f]+))?(?: err=(\S+))?")
ACK_RE = re.compile(r"\[ACK\] dump frame (\d+)")
ERR_RE = re.compile(r"\[ERR\]")


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def parse_frame_log(text: str) -> tuple[int, int, bytes, int]:
    lines = [ln.strip() for ln in text.splitlines()]
    collecting = False
    hex_parts: list[str] = []
    width = height = nbytes = 0

    for line in lines:
        if not line:
            continue
        if ERR_RE.search(line) and not collecting:
            raise RuntimeError(line)

        m = BEGIN_RE.search(line)
        if m:
            collecting = True
            hex_parts.clear()
            width = int(m.group(2))
            height = int(m.group(3))
            nbytes = int(m.group(4))
            continue

        if not collecting:
            continue

        end = END_RE.search(line)
        if end:
            if end.group(1) != "1":
                raise RuntimeError(f"Firmware dump failed: {line}")
            raw_hex = "".join(hex_parts)
            if len(raw_hex) != nbytes * 2:
                raise RuntimeError(
                    f"Hex length mismatch: got {len(raw_hex) // 2} B, expected {nbytes}"
                )
            data = bytes.fromhex(raw_hex)
            reported = int(end.group(2), 16) if end.group(2) else None
            calc = crc16_ccitt(data)
            if reported is not None and reported != calc:
                raise RuntimeError(f"CRC mismatch: fw={reported:04X} host={calc:04X}")
            return width, height, data, calc

        if re.fullmatch(r"[0-9A-Fa-f]+", line):
            hex_parts.append(line)

    raise RuntimeError("Не знайдено повний FRAME_BEGIN…FRAME_END у логу")


def write_outputs(out: Path, width: int, height: int, data: bytes, crc: int) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    raw_path = out.with_suffix(".raw")
    pgm_path = out.with_suffix(".pgm")
    raw_path.write_bytes(data)
    pgm_path.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + data)
    print(f"[host] wrote {raw_path} ({len(data)} B), CRC=0x{crc:04X}")
    print(f"[host] wrote {pgm_path}")
    try:
        from PIL import Image

        png_path = out.with_suffix(".png")
        Image.frombytes("L", (width, height), data).save(png_path)
        print(f"[host] wrote {png_path}")
    except ImportError:
        print("[host] Pillow not installed — skip PNG (pip install pillow)")


def open_nucleo_port(port: str, baud: int, boot_wait: float, verbose: bool):
    """
    Nucleo/ST-Link VCP при open часто робить reset МК і інколи
    перераховує USB. Тому: open → close → пауза → open знову → чекати boot.
    """
    import serial

    print(f"[host] opening {port} (1st, may reset MCU)...")
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.4
    ser.dsrdtr = False
    ser.rtscts = False
    ser.open()
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass
    time.sleep(0.3)
    ser.close()

    time.sleep(1.5)
    print(f"[host] reopening {port} after USB settle...")
    for attempt in range(10):
        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = baud
            ser.timeout = 0.4
            ser.dsrdtr = False
            ser.rtscts = False
            ser.open()
            try:
                ser.dtr = False
                ser.rts = False
            except Exception:
                pass
            break
        except serial.SerialException as exc:
            if attempt == 9:
                raise
            if verbose:
                print(f"[host] retry open ({exc})")
            time.sleep(0.5)

    print(f"[host] waiting {boot_wait:.0f}s for firmware boot/self-test...")
    deadline = time.time() + boot_wait
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk and verbose:
            text = chunk.decode("ascii", errors="replace")
            for ln in text.splitlines():
                ln = ln.strip()
                if ln:
                    print(f"[boot] {ln}")
    ser.reset_input_buffer()
    return ser


def wait_cli_ready(ser, timeout_s: float, verbose: bool) -> None:
    for attempt in range(3):
        ser.reset_input_buffer()
        ser.write(b"help\r\n")
        ser.flush()
        if verbose:
            print(f"[host] probe help (try {attempt + 1}/3)")
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                # також спробувати сирі байти
                raw = ser.read(64)
                if raw and verbose:
                    print(f"[raw] {raw!r}")
                continue
            if verbose:
                print(f"[rx] {line}")
            if (
                "Commands:" in line
                or line.startswith("  help")
                or "Incident Recorder" in line
                or "[OK]" in line
            ):
                time.sleep(0.4)
                # злити хвіст help
                end = time.time() + 0.8
                while time.time() < end:
                    ser.readline()
                return
        time.sleep(0.5)
    raise TimeoutError(
        "CLI не відповідає на 'help' навіть після reset/boot wait.\n"
        "Зробіть так:\n"
        "  1) Відкрийте PuTTY на цьому порту\n"
        "  2) Введіть: dump frame 0\n"
        "  3) Збережіть вивід у frame.txt (від FRAME_BEGIN до FRAME_END)\n"
        "  4) python3 tools/dump_frame.py --from-log frame.txt --out docs/metrics/frame0"
    )


def read_frame(ser, slot: int, timeout_s: float, verbose: bool) -> tuple[int, int, bytes, int]:
    ser.write(f"dump frame {slot}\r\n".encode("ascii"))
    ser.flush()
    print(f"[host] sent: dump frame {slot}")

    deadline = time.time() + timeout_s
    collecting = False
    saw_ack = False
    hex_parts: list[str] = []
    width = height = nbytes = 0
    last_lines: list[str] = []
    bytes_got = 0
    last_progress = 0.0
    buf = ""

    while time.time() < deadline:
        chunk = ser.read(512)
        if not chunk:
            continue
        buf += chunk.decode("ascii", errors="replace")
        while "\n" in buf or "\r" in buf:
            # нормалізувати рядки
            buf = buf.replace("\r\n", "\n").replace("\r", "\n")
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            last_lines.append(line)
            if len(last_lines) > 12:
                last_lines.pop(0)

            if ACK_RE.search(line):
                saw_ack = True
                print(f"[host] {line}")
                continue
            if ERR_RE.search(line) and not collecting:
                raise RuntimeError(line)

            m = BEGIN_RE.search(line)
            if m:
                collecting = True
                hex_parts.clear()
                width = int(m.group(2))
                height = int(m.group(3))
                nbytes = int(m.group(4))
                bytes_got = 0
                print(f"[host] begin {width}x{height} bytes={nbytes}")
                continue

            if not collecting:
                if verbose:
                    print(f"[rx] {line}")
                continue

            end = END_RE.search(line)
            if end:
                if end.group(1) != "1":
                    raise RuntimeError(line)
                raw_hex = "".join(hex_parts)
                if len(raw_hex) != nbytes * 2:
                    raise RuntimeError(
                        f"Hex length mismatch: got {len(raw_hex) // 2}, expected {nbytes}"
                    )
                data = bytes.fromhex(raw_hex)
                reported = int(end.group(2), 16) if end.group(2) else None
                calc = crc16_ccitt(data)
                if reported is not None and reported != calc:
                    raise RuntimeError(f"CRC mismatch fw={reported:04X} host={calc:04X}")
                return width, height, data, calc

            if re.fullmatch(r"[0-9A-Fa-f]+", line):
                hex_parts.append(line)
                bytes_got += len(line) // 2
                now = time.time()
                if now - last_progress >= 1.0:
                    pct = 100.0 * bytes_got / nbytes if nbytes else 0.0
                    print(f"[host] progress {bytes_got}/{nbytes} ({pct:.0f}%)")
                    last_progress = now

    hint = "no ACK" if not saw_ack else ("no BEGIN" if not collecting else "no END")
    raise TimeoutError(f"Timeout ({hint}). Last:\n" + "\n".join(last_lines))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--boot-wait", type=float, default=8.0, help="Seconds to wait after MCU reset")
    ap.add_argument("--from-log", help="Parse FRAME dump saved from PuTTY instead of UART")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    out = Path(args.out)

    if args.from_log:
        text = Path(args.from_log).read_text(encoding="utf-8", errors="replace")
        width, height, data, crc = parse_frame_log(text)
        write_outputs(out, width, height, data, crc)
        return 0

    if not args.port:
        raise SystemExit("Потрібен --port або --from-log")

    try:
        import serial  # noqa: F401
    except ImportError as exc:
        raise SystemExit("pip install pyserial") from exc

    ser = open_nucleo_port(args.port, args.baud, args.boot_wait, args.verbose)
    try:
        print("[host] probing CLI...")
        wait_cli_ready(ser, timeout_s=4.0, verbose=args.verbose)
        print("[host] CLI ok")
        width, height, data, crc = read_frame(
            ser, args.slot, timeout_s=args.timeout, verbose=args.verbose
        )
    finally:
        ser.close()

    write_outputs(out, width, height, data, crc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
