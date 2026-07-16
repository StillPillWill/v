#!/usr/bin/env python3
import asyncio
import json
import math
import sys

# Requirements:
# pip install websockets

URI = "ws://localhost:8080/control"

async def run_controller():
    print(f"Connecting to Nexus-4 Robot API Bridge at {URI}...")
    try:
        import websockets
    except ImportError:
        print("Error: The 'websockets' library is required to run this script.", file=sys.stderr)
        print("Install it by running: pip install websockets", file=sys.stderr)
        return

    try:
        async with websockets.connect(URI) as websocket:
            print("Connected to simulation server successfully!")
            print("Running trajectory controller. Control mode is active.")
            print("Press Ctrl+C to terminate connection.\n")

            start_time = None
            
            # Simple Python-side joint tracking gains for demonstration (if we run Torque Mode)
            # CTC is running in the browser by default, but we can override it by streaming torques
            Kp = [15.0, 20.0, 15.0, 5.0]
            Kd = [4.0, 5.0, 3.0, 1.0]

            while True:
                # 1. Read telemetry frame from the browser simulation
                message = await websocket.recv()
                telemetry = json.loads(message)

                t = telemetry["timestamp"] / 1000.0  # Convert to seconds
                if start_time is None:
                    start_time = t
                elapsed = t - start_time

                positions = telemetry["positions"]      # Actual joint angles (rad)
                velocities = telemetry["velocities"]    # Actual joint velocities (rad/s)
                cartesian = telemetry["cartesian"]      # End effector X, Y, Z (mm) and Pitch (deg)

                # 2. Define a desired motion trajectory in Python
                # We will trace a circle in Cartesian space or sweep joint targets.
                # Let's sweep joint targets for simplicity and clarity.
                # Joint 1: base yaw sweeps back and forth
                # Joint 2/3: shoulder/elbow perform a smooth sinusoidal lift
                target_q = [
                    0.6 * math.sin(1.0 * elapsed),                  # Joint 1 (Base)
                    deg_to_rad(30) + 0.3 * math.cos(1.2 * elapsed),  # Joint 2 (Shoulder)
                    deg_to_rad(60) + 0.2 * math.sin(1.5 * elapsed),  # Joint 3 (Elbow)
                    deg_to_rad(-10) + 0.15 * math.sin(1.0 * elapsed) # Joint 4 (Wrist)
                ]

                # We can control the robot in two ways:
                # Mode A: Set Target Position (the browser simulation runs the high-fidelity CTC loop)
                # Mode B: Set Torques directly (we calculate control torques in Python and override)
                
                # --- Choose Mode ---
                control_mode = "target"  # Change to "torque" to test direct torque control!
                
                if control_mode == "target":
                    # Stream desired joint angles back to simulator
                    command = {
                        "type": "targets",
                        "data": target_q
                    }
                else:
                    # Direct Torque Mode (Calculate PD torque here in Python)
                    torques = []
                    for i in range(4):
                        error = target_q[i] - positions[i]
                        # Handle base joint angular wrapping
                        if i == 0:
                            error = math.atan2(math.sin(error), math.cos(error))
                        
                        d_error = 0.0 - velocities[i] # Target velocity is 0 for simplicity
                        torque = Kp[i] * error + Kd[i] * d_error
                        torques.append(torque)
                    
                    command = {
                        "type": "torques",
                        "data": torques
                    }

                # Send command back to simulation
                await websocket.send(json.dumps(command))

                # Display stats in terminal
                print(f"\rTime: {elapsed:5.2f}s | "
                      f"End Effector (X, Y, Z): ({cartesian['x']:5.1f}, {cartesian['y']:5.1f}, {cartesian['z']:5.1f}) | "
                      f"Base Err: {rad_to_deg(target_q[0] - positions[0]):5.2f}°", end="")
                
                # Slow down terminal output slightly, though connection is driven by telemetry frequency
                await asyncio.sleep(0.001)

    except websockets.exceptions.ConnectionClosed:
        print("\nConnection to server closed.")
    except KeyboardInterrupt:
        print("\nTerminating controller client.")

def deg_to_rad(deg):
    return (deg * math.PI) / 180.0

def rad_to_deg(rad):
    return (rad * 180.0) / math.PI

if __name__ == "__main__":
    try:
        asyncio.run(run_controller())
    except KeyboardInterrupt:
        pass
