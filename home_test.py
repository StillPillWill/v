import serial
import serial.tools.list_ports
import time

def home_printer():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    print(f"Found ports: {ports}")
    
    for port in ports:
        try:
            print(f"Trying {port} at 115200 baud...")
            s = serial.Serial(port, 115200, timeout=2)
            time.sleep(2)
            
            # Send M115 to get firmware info (identifies if it's a 3D printer/Marlin)
            s.write(b"M115\n")
            
            response = ""
            start_time = time.time()
            while time.time() - start_time < 2:
                line = s.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[{port}] {line}")
                    response += line
                if "FIRMWARE_NAME" in response or "ok" in line.lower():
                    print(f"\n---> Printer found on {port}!")
                    
                    print("Sending M999 (Clear Errors)...")
                    s.write(b"M999\n")
                    time.sleep(0.5)
                    
                    print("Sending G90 (Absolute Mode)...")
                    s.write(b"G90\n")
                    time.sleep(0.5)
                    
                    print("Sending G28 (Home All Axes)...")
                    s.write(b"G28\n")
                    
                    print("Waiting for response...")
                    while True:
                        line = s.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            print(f"Printer: {line}")
                        if line == "ok" or "ok" in line.lower():
                            print("\n=== Printer homed successfully! ===")
                            s.close()
                            return
            
            s.close()
        except Exception as e:
            print(f"Error on {port}: {e}")

if __name__ == "__main__":
    home_printer()
