#!/usr/bin/env python3
import asyncio
import json
import math
import sys
import os

# --- Hardware Integration Instructions ---
# To connect to a physical robot arm:
# 1. Install pyserial: pip install pyserial
# 2. Set the SERIAL_PORT variable below (e.g. "COM3" on Windows, "/dev/ttyUSB0" on Linux)
# 3. Upload the matching Arduino/ESP32 firmware (provided in walkthrough.md) to your microcontroller.

SERIAL_PORT = None  # Change to your COM port (e.g., "COM3") to connect real hardware
BAUD_RATE = 115200
WS_URI = "ws://localhost:8080/control"

# Try importing pyserial for hardware connection
serial_connection = None
if SERIAL_PORT:
    try:
        import serial
        serial_connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"[HARDWARE] Opened Serial connection to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except ImportError:
        print("[WARNING] 'pyserial' not installed. Running in MOCK hardware mode.")
    except Exception as e:
        print(f"[ERROR] Failed to open serial port {SERIAL_PORT}: {e}")
        print("Running in MOCK hardware mode.")
else:
    print("[HARDWARE] No SERIAL_PORT specified. Running in MOCK hardware mode.")


def rad_to_deg(rad):
    return (rad * 180.0) / math.pi


def apply_safety_limits(j1, j2, j3, j4):
    """
    Applies joint saturation limits to protect the physical robot's actuators.
    Converts raw kinematics (centered at 0) to standard 0-180 servo degrees.
    """
    # 1. Base Yaw (J1): Map -90 to +90 rad -> 0 to 180 deg (centered at 90)
    servo1 = int(rad_to_deg(j1) + 90)
    servo1 = max(0, min(180, servo1))

    # 2. Shoulder Pitch (J2): Map -90 to +90 rad -> 0 to 180 deg (centered at 90)
    servo2 = int(rad_to_deg(j2) + 90)
    servo2 = max(10, min(170, servo2)) # Leave safety margin

    # 3. Elbow Pitch (J3): Map -90 to +90 rad -> 0 to 180 deg (centered at 90)
    servo3 = int(rad_to_deg(j3) + 90)
    servo3 = max(10, min(170, servo3))

    # 4. Wrist Pitch (J4): Map -90 to +90 rad -> 0 to 180 deg (centered at 90)
    servo4 = int(rad_to_deg(j4) + 90)
    servo4 = max(0, min(180, servo4))

    return servo1, servo2, servo3, servo4


async def run_robot_bridge():
    try:
        import websockets
    except ImportError:
        print("Error: The 'websockets' library is required to run this bridge.", file=sys.stderr)
        print("Install it by running: pip install websockets", file=sys.stderr)
        return

    print(f"\n[BRIDGE] Connecting to visualizer API at {WS_URI}...")
    try:
        async with websockets.connect(WS_URI) as websocket:
            print("[BRIDGE] Connected to simulation server successfully!")
            print("[BRIDGE] Streaming tracking targets to hardware. Press Ctrl+C to stop.\n")

            # Store last known positions for smooth deceleration / hold if connection drops
            last_servos = (90, 90, 90, 90)
            last_grip = 0

            while True:
                # 1. Receive telemetry frame from the visualizer
                message = await websocket.recv()
                telemetry = json.loads(message)

                # Extract joint angles, gripper, and cartesian feedback
                positions = telemetry.get("positions", [0, 0, 0, 0])
                fist_closed = telemetry.get("fistClosed", False)
                cartesian = telemetry.get("cartesian", {"x": 0, "y": 0, "z": 0, "pitch": 0})

                # 2. Convert radians to physical servo degrees and clamp to safe boundaries
                s1, s2, s3, s4 = apply_safety_limits(
                    positions[0], positions[1], positions[2], positions[3]
                )
                grip_state = 1 if fist_closed else 0  # 1 = Close Gripper, 0 = Open Gripper

                # 3. Format hardware control packet
                # Standard packet layout: "R:<joint1>,<joint2>,<joint3>,<joint4>,<grip_state>\n"
                packet = f"R:{s1},{s2},{s3},{s4},{grip_state}\n"

                # 4. Transmit to physical robot over Serial link
                if serial_connection and serial_connection.is_open:
                    serial_connection.write(packet.encode("utf-8"))
                    serial_connection.flush()
                    hw_status = "ACTIVE"
                else:
                    hw_status = "MOCKED"

                # 5. Render live bridge terminal dashboard
                sys.stdout.write(
                    f"\r[CMD] Joint Degs: ({s1:3d}°, {s2:3d}°, {s3:3d}°, {s4:3d}°) | "
                    f"Grip: {'CLOSED (GRASP)' if grip_state == 1 else 'OPEN  (IDLE) '} | "
                    f"Coord: ({cartesian['x']:5.1f}, {cartesian['y']:5.1f}, {cartesian['z']:5.1f}) | "
                    f"HW: {hw_status}"
                )
                sys.stdout.flush()

                # Yield control to event loop briefly
                await asyncio.sleep(0.001)

    except websockets.exceptions.ConnectionClosed:
        print("\n[BRIDGE] WebSocket connection to server lost.")
    except Exception as e:
        print(f"\n[BRIDGE] Error: {e}")
    finally:
        # Failsafe: Send safe park command on shutdown
        print("\n[FAILSAFE] Sending safe park command to robot...")
        park_packet = "R:90,90,90,90,0\n"
        if serial_connection and serial_connection.is_open:
            serial_connection.write(park_packet.encode("utf-8"))
            serial_connection.close()
            print("[HARDWARE] Serial connection closed safely.")


if __name__ == "__main__":
    try:
        asyncio.run(run_robot_bridge())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Bridge terminated by user.")
