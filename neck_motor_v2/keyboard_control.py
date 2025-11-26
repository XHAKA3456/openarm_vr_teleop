#!/usr/bin/env python3
"""
Keyboard control for neck motors.
- Up/Down arrows: Control nod (ID 6) - up/down nod
- Left/Right arrows: Control rotate (ID 5) - left/right rotation
- W/S keys: Control tilt (ID 4) - left/right tilt
- Press 'q' or Ctrl+C to quit
"""

import sys
import termios
import tty
from controller import STS3215Controller

# Initial positions (measured from home position)
INITIAL_NOD = 2722     # Up/Down nod (ID 6)
INITIAL_ROTATE = 3096  # Left/Right rotation (ID 5)
INITIAL_TILT = 1243    # Left/Right tilt (ID 4)

# Step size
STEP = 5

# Position limits
MIN_POS = 0
MAX_POS = 4095


def get_key():
    """Read a single keypress from stdin."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # Handle arrow keys (escape sequences)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                return '\x1b[' + ch3
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def clamp(value, min_val, max_val):
    """Clamp value between min and max."""
    return max(min_val, min(max_val, value))


def main():
    controller = STS3215Controller(port="/dev/ttyUSB0")
    controller.open()
    controller.enable_torque(True)

    # Set initial positions
    nod_pos = INITIAL_NOD        # Up/Down nod
    rotate_pos = INITIAL_ROTATE  # Left/Right rotation
    tilt_pos = INITIAL_TILT      # Left/Right tilt

    print("Neck Motor Keyboard Control")
    print("===========================")
    print(f"Initial positions:")
    print(f"  nod (ID 6):    {nod_pos} (Up/Down nod)")
    print(f"  rotate (ID 5): {rotate_pos} (Left/Right rotation)")
    print(f"  tilt (ID 4):   {tilt_pos} (Left/Right tilt)")
    print()
    print("Controls:")
    print("  Up/Down arrows:    Control nod (ID 6)")
    print("  Left/Right arrows: Control rotate (ID 5)")
    print("  W/S keys:          Control tilt (ID 4)")
    print("  'q':               Quit")
    print()

    # Move to initial positions
    controller.command_positions({
        "nod": nod_pos,
        "rotate": rotate_pos,
        "tilt": tilt_pos
    })

    try:
        while True:
            key = get_key()

            if key == 'q' or key == '\x03':  # 'q' or Ctrl+C
                break

            # Arrow keys
            elif key == '\x1b[A':  # Up arrow
                nod_pos = clamp(nod_pos - STEP, MIN_POS, MAX_POS)
                print(f"↑ nod (ID 6): {nod_pos}")
            elif key == '\x1b[B':  # Down arrow
                nod_pos = clamp(nod_pos + STEP, MIN_POS, MAX_POS)
                print(f"↓ nod (ID 6): {nod_pos}")
            elif key == '\x1b[D':  # Left arrow
                rotate_pos = clamp(rotate_pos - STEP, MIN_POS, MAX_POS)
                print(f"← rotate (ID 5): {rotate_pos}")
            elif key == '\x1b[C':  # Right arrow
                rotate_pos = clamp(rotate_pos + STEP, MIN_POS, MAX_POS)
                print(f"→ rotate (ID 5): {rotate_pos}")
            # W/S keys for tilt
            elif key == 'w' or key == 'W':
                tilt_pos = clamp(tilt_pos - STEP, MIN_POS, MAX_POS)
                print(f"W tilt (ID 4): {tilt_pos}")
            elif key == 's' or key == 'S':
                tilt_pos = clamp(tilt_pos + STEP, MIN_POS, MAX_POS)
                print(f"S tilt (ID 4): {tilt_pos}")
            else:
                continue

            # Send command to motors
            controller.command_positions({
                "nod": nod_pos,
                "rotate": rotate_pos,
                "tilt": tilt_pos
            })

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        print("\nFinal positions:")
        print(f"  nod (ID 6):    {nod_pos}")
        print(f"  rotate (ID 5): {rotate_pos}")
        print(f"  tilt (ID 4):   {tilt_pos}")
        controller.enable_torque(False)
        controller.close()


if __name__ == "__main__":
    main()
