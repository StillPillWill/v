import asyncio
import json
import sys

URI = "ws://localhost:8080/control"

async def read_telemetry():
    try:
        import websockets
    except ImportError:
        print("Install websockets first: pip install websockets")
        return

    print("Connecting to WebSocket to check telemetry data...")
    try:
        async with websockets.connect(URI) as ws:
            print("Connected. Reading 10 frames of telemetry:\n")
            for i in range(15):
                msg = await ws.recv()
                data = json.loads(msg)
                
                pos_deg = [round(p * 180 / 3.14159, 2) for p in data["positions"]]
                vel_deg = [round(v * 180 / 3.14159, 2) for v in data["velocities"]]
                trq = [round(t, 2) for t in data["appliedTorques"]]
                cart = data["cartesian"]
                
                print(f"Frame {i+1:2d} | Pos: {pos_deg} | Vel: {vel_deg} | Torques: {trq} | EE: ({cart['x']:.1f}, {cart['y']:.1f}, {cart['z']:.1f})")
    except Exception as e:
        print("Error reading websocket:", e)

if __name__ == "__main__":
    asyncio.run(read_telemetry())
