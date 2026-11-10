#!/usr/bin/env python3
import os
import sys

FORBIDDEN_PATTERNS = [
    "stm32f4xx_hal.h",
    "main.h",
    "stm32f4",
    "HAL_"
]

SCAN_DIRS = [
    os.path.join("firmware", "Devices"),
    os.path.join("firmware", "App")
]

violations = []

for s_dir in SCAN_DIRS:
    if not os.path.exists(s_dir):
        continue
    for root, _, files in os.walk(s_dir):
        for file in files:
            if file.endswith((".c", ".h")):
                fpath = os.path.join(root, file)
                with open(fpath, "r", encoding="utf-8", errors="ignore") as f:
                    for line_no, line in enumerate(f, 1):
                        for pat in FORBIDDEN_PATTERNS:
                            if pat in line:
                                violations.append(f"{fpath}:{line_no} -> містить заборонену залежність '{pat}'")

if violations:
    print("[FAIL] Порушення шаруватої архітектури! Знайдено прямі прив'язки до HAL:")
    for v in violations:
        print("  " + v)
    sys.exit(1)
else:
    print("[PASS] Перевірка архітектури успішна: Devices/ та App/ чисті від вендорського HAL.")
    sys.exit(0)