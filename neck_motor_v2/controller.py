#!/usr/bin/env python3
"""
Standalone helper for driving up to three Feetech STS3215 motors without depending on lerobot.

The implementation wraps the vendor SDK (feetech-servo-sdk / scservo_sdk) and exposes a tiny API that
opens /dev/ttyUSB* ports, reads the current joint state, and writes target positions or torque flags.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Dict, Sequence, Union

try:
    import scservo_sdk as scs
except ImportError as exc:
    raise ImportError(
        "feetech-servo-sdk is required. Install it with `pip install feetech-servo-sdk`."
    ) from exc


# Control-table constants for STS/SMS series (protocol 0)
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56

BYTES_TORQUE_ENABLE = 1
BYTES_GOAL_POSITION = 2
BYTES_PRESENT_POSITION = 2

STS3215_PROTOCOL_VERSION = 0
STS3215_DEFAULT_BAUD = 1_000_000


@dataclass(frozen=True)
class MotorConfig:
    name: str
    motor_id: int


NECK_MOTORS: tuple[MotorConfig, ...] = (
    MotorConfig("tilt", 4),    # Left/Right tilt
    MotorConfig("rotate", 5),  # Left/Right rotation
    MotorConfig("nod", 6),     # Up/Down nod
)


class STS3215Controller:
    """
    Thin wrapper around scservo_sdk for a neck rig with up to three STS3215 motors.

    Each controller keeps GroupSyncRead/Write objects configured for the common addresses so that callers can
    read all motors at once and stream new goal positions every control loop iteration.
    """

    def __init__(
        self,
        port: str,
        motor_ids: Sequence[int] | None = None,
        motors: Sequence[MotorConfig] | None = None,
        baudrate: int = STS3215_DEFAULT_BAUD,
        protocol_version: int = STS3215_PROTOCOL_VERSION,
    ) -> None:
        motor_cfgs: Sequence[MotorConfig] | None = motors
        if motor_cfgs is None:
            if motor_ids is not None:
                motor_cfgs = [MotorConfig(f"motor_{idx}", idx) for idx in motor_ids]
            else:
                motor_cfgs = NECK_MOTORS
        if not motor_cfgs:
            raise ValueError("Specify at least one motor via `motors` or `motor_ids`.")

        names = [cfg.name for cfg in motor_cfgs]
        ids = [cfg.motor_id for cfg in motor_cfgs]
        if len(set(names)) != len(names):
            raise ValueError("Motor names must be unique.")
        if len(set(ids)) != len(ids):
            raise ValueError("Motor IDs must be unique.")

        self.port = port
        self.motor_configs = tuple(motor_cfgs)
        self.motor_ids = tuple(cfg.motor_id for cfg in self.motor_configs)
        self.name_to_id = {cfg.name: cfg.motor_id for cfg in self.motor_configs}
        self.id_to_name = {cfg.motor_id: cfg.name for cfg in self.motor_configs}
        self.port_handler = scs.PortHandler(port)
        self.packet_handler = scs.PacketHandler(protocol_version)
        self.group_read = scs.GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_POSITION,
            BYTES_PRESENT_POSITION,
        )
        self.group_write = scs.GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            BYTES_GOAL_POSITION,
        )
        self.baudrate = baudrate
        self._add_sync_params()

    def _add_sync_params(self) -> None:
        self.group_read.clearParam()
        for motor_id in self.motor_ids:
            added = self.group_read.addParam(motor_id)
            if not added:
                raise RuntimeError(f"Failed to register motor {motor_id} for sync read.")

    def open(self) -> None:
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port {self.port}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"Failed to set baudrate {self.baudrate}")

    def close(self) -> None:
        if self.port_handler.is_open:
            self.port_handler.closePort()

    def enable_torque(self, enable: bool) -> None:
        value = 1 if enable else 0
        for motor_id in self.motor_ids:
            self._write_byte(ADDR_TORQUE_ENABLE, motor_id, value)

    def _write_byte(self, address: int, motor_id: int, value: int) -> None:
        error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            motor_id,
            address,
            value,
        )[1]
        if error != 0:
            raise RuntimeError(f"write1Byte error={error} on motor {motor_id}")

    def read_positions(self) -> Dict[str, int]:
        """Return raw encoder positions keyed by motor name."""
        self.group_read.txRxPacket()
        values: Dict[str, int] = {}
        for motor_id in self.motor_ids:
            if not self.group_read.isAvailable(motor_id, ADDR_PRESENT_POSITION, BYTES_PRESENT_POSITION):
                raise RuntimeError(f"No data available for motor {motor_id}")
            pos = self.group_read.getData(motor_id, ADDR_PRESENT_POSITION, BYTES_PRESENT_POSITION)
            values[self.id_to_name[motor_id]] = pos
        return values

    def read_positions_raw(self) -> Dict[int, int]:
        """Return raw encoder positions keyed by ID."""
        named = self.read_positions()
        return {self.name_to_id[name]: value for name, value in named.items()}

    def command_positions(self, goal_positions: Dict[Union[str, int], int]) -> None:
        """
        Write goal positions for one or more motors.

        Values must be raw encoder ticks (0-4095 for STS3215). Missing motors keep their previous goals.
        """
        self.group_write.clearParam()
        for key, value in goal_positions.items():
            motor_id = self._resolve_motor_id(key)
            if motor_id not in self.motor_ids:
                raise KeyError(f"Motor ID {motor_id} was not registered with this controller.")
            packet = struct.pack("<H", value)
            added = self.group_write.addParam(motor_id, packet)
            if not added:
                raise RuntimeError(f"Failed to add motor {motor_id} goal to sync write buffer.")
        if goal_positions:
            dxl_comm_result = self.group_write.txPacket()
            if dxl_comm_result != scs.COMM_SUCCESS:
                msg = self.packet_handler.getTxRxResult(dxl_comm_result)
                raise RuntimeError(f"Sync write failed: {msg}")

    def _resolve_motor_id(self, key: Union[str, int]) -> int:
        if isinstance(key, str):
            if key not in self.name_to_id:
                raise KeyError(f"Unknown motor name '{key}'. Known motors: {list(self.name_to_id)}")
            return self.name_to_id[key]
        return int(key)


def example() -> None:
    """
    Demonstrates how to use the controller. Replace the IDs and device node to match your setup.
    """

    controller = STS3215Controller(port="/dev/ttyUSB0")

    controller.open()
    controller.enable_torque(True)

    try:
        positions = controller.read_positions()
        print("Current positions:", positions)

        # Move motors a little bit as a demo (keyed by human-friendly names).
        demo_targets = {name: (pos + 200) % 4096 for name, pos in positions.items()}
        controller.command_positions(demo_targets)
        print("Commanded demo positions:", demo_targets)
    finally:
        controller.enable_torque(False)
        controller.close()


if __name__ == "__main__":
    example()
