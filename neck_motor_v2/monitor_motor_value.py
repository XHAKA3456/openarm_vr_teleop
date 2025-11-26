#!/usr/bin/env python3
"""
Live monitor for the STS3215 neck rig. Continuously reads motor positions and shows them in a table.

Press Ctrl+C to exit.
"""

from __future__ import annotations

import math
import sys
import time
from typing import Iterable

from controller import NECK_MOTORS, STS3215Controller

REF_TICKS = 2048  # middle of the 12-bit range
FULL_REV_TICKS = 4096


def ticks_to_degrees(ticks: int) -> float:
    return (ticks - REF_TICKS) * 360.0 / FULL_REV_TICKS


def build_table(rows: list[tuple[str, int, int, float]]) -> str:
    headers = ("Name", "ID", "Raw", "Deg")
    data = [headers] + [(name, str(mid), str(raw), f"{deg:+6.1f}") for name, mid, raw, deg in rows]

    col_widths = [max(len(row[col]) for row in data) for col in range(len(headers))]

    lines = []
    header_line = " | ".join(val.ljust(col_widths[idx]) for idx, val in enumerate(data[0]))
    sep_line = "-+-".join("-" * col_widths[idx] for idx in range(len(headers)))
    lines.extend([header_line, sep_line])

    for row in data[1:]:
        lines.append(" | ".join(val.ljust(col_widths[idx]) for idx, val in enumerate(row)))

    return "\n".join(lines)


def clear_screen() -> None:
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


def main() -> None:
    controller = STS3215Controller(port="/dev/ttyUSB0")
    controller.open()
    controller.enable_torque(False)

    print("Live monitoring started. Move the motors by hand to see raw ticks change. Ctrl+C to exit.")
    time.sleep(1.0)

    try:
        while True:
            positions = controller.read_positions()
            rows = []
            for cfg in controller.motor_configs:
                raw = positions.get(cfg.name, 0)
                rows.append((cfg.name, cfg.motor_id, raw, ticks_to_degrees(raw)))

            clear_screen()
            print("Neck Motor Monitor")
            print(build_table(rows))
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping monitor...")
    finally:
        controller.close()


if __name__ == "__main__":
    main()
