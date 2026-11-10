#!/usr/bin/env python3
"""Розбір UART-логу Lab 3: EVT / STAT / LOAD / IMU_JITTER."""

from __future__ import annotations

import argparse
import re
import statistics
import sys
from pathlib import Path

EVT_RE = re.compile(r"EVT t_us=(\d+) name=(\S+)")
STAT_RE = re.compile(
    r"STAT name=(\S+) n=(\d+)(?: min_us=(\d+) avg_us=(\d+) max_us=(\d+))?"
)
LOAD_RE = re.compile(r"LOAD pct=(\d+)")
JITTER_RE = re.compile(r"IMU_JITTER min_us=(-?\d+) max_us=(-?\d+) missed=(\d+)")


def latencies(events: list[tuple[int, str]], start: str, end: str) -> list[int]:
    out: list[int] = []
    t0 = None
    for t, name in events:
        if name == start:
            t0 = t
        elif name == end and t0 is not None:
            out.append(t - t0)
            t0 = None
    return out


def summarize(xs: list[int], label: str) -> None:
    if not xs:
        print(f"{label}: (no samples)")
        return
    print(
        f"{label}: n={len(xs)} min={min(xs)} median={statistics.median(xs):.0f} "
        f"max={max(xs)} us"
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("log", type=Path)
    args = ap.parse_args()
    text = args.log.read_text(encoding="utf-8", errors="replace")

    events: list[tuple[int, str]] = []
    for line in text.splitlines():
        m = EVT_RE.search(line)
        if m:
            events.append((int(m.group(1)), m.group(2)))
            continue
        m = STAT_RE.search(line)
        if m and m.group(3):
            print(
                f"STAT {m.group(1)}: n={m.group(2)} "
                f"min={m.group(3)} avg={m.group(4)} max={m.group(5)} us"
            )
            continue
        m = LOAD_RE.search(line)
        if m:
            print(f"LOAD: {m.group(1)}%")
            continue
        m = JITTER_RE.search(line)
        if m:
            print(
                f"IMU_JITTER: min={m.group(1)} max={m.group(2)} us missed={m.group(3)}"
            )

    print("--- EVT latencies ---")
    for a, b in [
        ("btn_press", "incident"),
        ("incident", "capture_start"),
        ("capture_start", "capture_done"),
        ("flash_write_start", "flash_write_done"),
        ("btn_press", "capture_start"),
    ]:
        summarize(latencies(events, a, b), f"{a} -> {b}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
