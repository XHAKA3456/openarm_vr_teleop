#!/usr/bin/env python3
"""
Small CLI to scan Feetech STS/SMS servos on a serial bus and assign them new IDs.

This is a lightweight alternative to lerobot's calibration tools when you only need to prep raw motors.
"""

from __future__ import annotations

import argparse
from typing import List, Tuple

try:
    import scservo_sdk as scs
except ImportError as exc:
    raise ImportError(
        "feetech-servo-sdk is required. Install it with `pip install feetech-servo-sdk`."
    ) from exc


ADDR_ID = 5
PROTOCOL_VERSION = 0  # STS/SMS series default
DEFAULT_BAUDRATE = 1_000_000
MIN_ID = getattr(scs, "MIN_ID", 0)
MAX_ID = getattr(scs, "MAX_ID", 253)


def open_port(port: str, baudrate: int) -> scs.PortHandler:
    handler = scs.PortHandler(port)
    if not handler.openPort():
        raise RuntimeError(f"Failed to open port {port}")
    if not handler.setBaudRate(baudrate):
        raise RuntimeError(f"Failed to set baudrate {baudrate}")
    return handler


def scan_ids(port_handler: scs.PortHandler, packet_handler: scs.PacketHandler) -> List[Tuple[int, int]]:
    motors: List[Tuple[int, int]] = []
    for motor_id in range(MIN_ID, MAX_ID + 1):
        model_nb, comm, error = packet_handler.ping(port_handler, motor_id)
        if comm == scs.COMM_SUCCESS and error == 0:
            motors.append((motor_id, model_nb))
    return motors


def detect_single_motor(
    port_handler: scs.PortHandler,
    packet_handler: scs.PacketHandler,
) -> int:
    motors = scan_ids(port_handler, packet_handler)
    if not motors:
        raise RuntimeError("No motor detected on the bus. Connect exactly one motor and try again.")
    if len(motors) > 1:
        ids = ", ".join(str(motor_id) for motor_id, _ in motors)
        raise RuntimeError(
            f"Multiple motors detected ({ids}). Disconnect all but one motor before assigning the next ID."
        )
    return motors[0][0]


def assign_id(
    port_handler: scs.PortHandler,
    packet_handler: scs.PacketHandler,
    current_id: int,
    new_id: int,
) -> None:
    if new_id < MIN_ID or new_id > MAX_ID:
        raise ValueError(f"new_id must be between {MIN_ID} and {MAX_ID}")

    if current_id == new_id:
        print("New ID is identical to current ID; nothing to do.")
        return

    model_nb, comm, error = packet_handler.ping(port_handler, current_id)
    if comm != scs.COMM_SUCCESS or error != 0:
        raise RuntimeError(f"Motor {current_id} not responding (comm={comm}, error={error})")
    print(f"Found motor {current_id} (model {model_nb}) → assigning new ID {new_id}")

    comm, error = packet_handler.write1ByteTxRx(port_handler, current_id, ADDR_ID, new_id)
    if comm != scs.COMM_SUCCESS:
        raise RuntimeError(f"Failed to write ID (comm={comm})")
    if error != 0:
        raise RuntimeError(f"Motor reported error while writing ID: {packet_handler.getRxPacketError(error)}")

    # verify with ping on new id
    _, comm, error = packet_handler.ping(port_handler, new_id)
    if comm != scs.COMM_SUCCESS or error != 0:
        raise RuntimeError("Verification ping failed on the new ID.")
    print(f"✅ Motor now responds on ID {new_id}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan or assign IDs to Feetech STS/SMS motors.")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial device path")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="Bus baudrate")
    parser.add_argument("--scan", action="store_true", help="List all motors currently responding")
    parser.add_argument("--current-id", type=int, help="Existing motor ID")
    parser.add_argument("--new-id", type=int, help="Target ID to program")
    parser.add_argument(
        "--sequence",
        type=int,
        nargs="+",
        metavar="ID",
        help="Interactive mode: list of new IDs to assign sequentially (press Enter between motors).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    packet_handler = scs.PacketHandler(PROTOCOL_VERSION)
    port_handler = open_port(args.port, args.baudrate)

    try:
        if args.sequence:
            print("Sequential ID assignment mode.")
            print("Please connect one motor at a time. For each step, connect the motor, then press Enter.")
            for next_id in args.sequence:
                input(f"\nReady to assign ID {next_id}. Connect the motor and press Enter...")
                current_id = detect_single_motor(port_handler, packet_handler)
                try:
                    assign_id(port_handler, packet_handler, current_id, next_id)
                except Exception as exc:
                    print(f"❌ Failed to assign ID {next_id}: {exc}")
                    raise
            print("\nAll IDs assigned. Disconnect the last motor before continuing.")
            return

        if args.scan:
            motors = scan_ids(port_handler, packet_handler)
            if not motors:
                print("No motors responded.")
            else:
                print("Detected motors (id → model):")
                for motor_id, model in motors:
                    print(f"  {motor_id:3d} → {model}")
            return

        if args.current_id is None or args.new_id is None:
            raise SystemExit("Specify --scan or both --current-id and --new-id.")

        assign_id(port_handler, packet_handler, args.current_id, args.new_id)

    finally:
        port_handler.closePort()


if __name__ == "__main__":
    main()
