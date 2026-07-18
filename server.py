import os
import sys
import time
import socket
import threading
import hashlib
import base64
import subprocess
import shutil

# --- Auto Dependency Checker ---
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial is missing. Attempting to install it via pip...")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
        import serial
        import serial.tools.list_ports
        print("pyserial successfully installed!")
    except Exception as e:
        print(f"Failed to install pyserial automatically: {e}")
        print("Please run: pip install pyserial")
        sys.exit(1)

import http.server
import socketserver

# --- Configurations ---
HTTP_PORT = 8000
WS_PORT = 8765
BAUD_RATE = 115200

# --- Serial Controller State ---
serial_port = None
serial_lock = threading.Lock()
in_flight = 0
max_in_flight = 1
serial_queue = []
serial_queue_cv = threading.Condition()
keep_running = True
printer_connected = False
printer_port_name = None

# --- Custom Static HTTP Server with JS MIME Type Enforcer ---
class CustomHTTPHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def guess_type(self, path):
        mimetype = super().guess_type(path)
        if path.endswith('.js'):
            return 'application/javascript'
        return mimetype

def start_http_server():
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", HTTP_PORT), CustomHTTPHandler) as httpd:
        print(f"HTTP Server running at http://localhost:{HTTP_PORT}")
        httpd.serve_forever()

# --- Lightweight Custom WebSocket Server ---
def compute_ws_accept(sec_key):
    magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
    hashed = hashlib.sha1((sec_key + magic).encode('utf-8')).digest()
    return base64.b64encode(hashed).decode('utf-8')

def parse_ws_frame(data):
    if len(data) < 2:
        return None, None
    second_byte = data[1]
    masked = (second_byte & 128) != 0
    payload_len = second_byte & 127
    
    idx = 2
    if payload_len == 126:
        if len(data) < 4: return None, None
        payload_len = int.from_bytes(data[2:4], byteorder='big')
        idx = 4
    elif payload_len == 127:
        if len(data) < 10: return None, None
        payload_len = int.from_bytes(data[2:10], byteorder='big')
        idx = 10
        
    if len(data) < idx + (4 if masked else 0) + payload_len:
        return None, None  # Incomplete frame
        
    if masked:
        mask_key = data[idx:idx+4]
        idx += 4
        payload = data[idx:idx+payload_len]
        decoded = bytearray(payload_len)
        for i in range(payload_len):
            decoded[i] = payload[i] ^ mask_key[i % 4]
        return decoded.decode('utf-8', errors='ignore'), idx + payload_len
    else:
        payload = data[idx:idx+payload_len]
        return payload.decode('utf-8', errors='ignore'), idx + payload_len

def make_ws_frame(text):
    payload = text.encode('utf-8')
    header = bytearray()
    header.append(129) # Fin + Text Frame (0x81)
    payload_len = len(payload)
    if payload_len <= 125:
        header.append(payload_len)
    elif payload_len <= 65535:
        header.append(126)
        header.extend(payload_len.to_bytes(2, byteorder='big'))
    else:
        header.append(127)
        header.extend(payload_len.to_bytes(8, byteorder='big'))
    return header + payload

# --- Printer Serial Thread Loops ---
def printer_reader_thread():
    global in_flight, serial_port, keep_running
    print("[SERIAL] Reader thread started.")
    buffer = ""
    while keep_running and serial_port and serial_port.is_open:
        try:
            if serial_port.in_waiting > 0:
                chars = serial_port.read(serial_port.in_waiting).decode('utf-8', errors='ignore')
                buffer += chars
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        # Print responses for visibility
                        if not line.startswith("ok") and not line.startswith("T:"):
                            print(f"[PRINTER] {line}")
                        if "ok" in line.lower():
                            with serial_lock:
                                in_flight = max(0, in_flight - 1)
                            # Wake up queue writer thread
                            with serial_queue_cv:
                                serial_queue_cv.notify()
            else:
                time.sleep(0.002)
        except Exception as e:
            print(f"[SERIAL READER ERROR] {e}")
            disconnect_printer()
            break
    print("[SERIAL] Reader thread stopped.")

def printer_writer_thread():
    global in_flight, serial_port, serial_queue, keep_running
    print("[SERIAL] Writer thread started.")
    while keep_running:
        cmd = None
        with serial_queue_cv:
            while keep_running and (len(serial_queue) == 0 or in_flight >= max_in_flight):
                serial_queue_cv.wait(timeout=0.1)
            if not keep_running:
                break
            if len(serial_queue) > 0:
                cmd = serial_queue.pop(0)
                in_flight += 1
                
        if cmd:
            try:
                serial_port.write((cmd + "\n").encode('utf-8'))
            except Exception as e:
                print(f"[SERIAL WRITER ERROR] {e}")
                disconnect_printer()
                with serial_lock:
                    in_flight = max(0, in_flight - 1)

    print("[SERIAL] Writer thread stopped.")

# --- Auto Connect Printer Function ---
def try_connect_printer():
    global serial_port, printer_connected, printer_port_name
    
    # Try current active COM ports
    ports = [p.device for p in serial.tools.list_ports.comports()]
    print(f"[SERIAL] Scanning COM ports: {ports}")
    
    # Check preferred ports first (highest priority USB serials on Windows/Mac/Linux)
    preferred_keywords = ['usbserial', 'usbmodem', 'ttyusb', 'ttyacm', 'com13', 'com5']
    preferred_ports = []
    other_ports = []
    for port in ports:
        is_preferred = False
        for kw in preferred_keywords:
            if kw in port.lower():
                is_preferred = True
                break
        if is_preferred:
            preferred_ports.append(port)
        else:
            other_ports.append(port)
            
    ports = preferred_ports + other_ports
            
    bauds = [115200, 250000]
    
    for port in ports:
        for baud in bauds:
            try:
                print(f"[SERIAL] Attempting to probe {port} at {baud} baud...")
                s = serial.Serial(port, baud, timeout=1.5)
                
                # Wait for printer to complete reboot cycle (some take longer)
                time.sleep(2.5) 
                
                # Clear read buffer
                if s.in_waiting > 0:
                    startup_text = s.read(s.in_waiting).decode('utf-8', errors='ignore')
                    print(f"[SERIAL] Startup text read: {startup_text.strip()}")
                    if "start" in startup_text.lower() or "marlin" in startup_text.lower() or "echo:" in startup_text.lower():
                        print(f"[SERIAL] Printer identified via startup text on {port}!")
                        serial_port = s
                        printer_port_name = port
                        printer_connected = True
                        break

                # Send empty command (newline) and query info
                s.write(b"\n")
                time.sleep(0.2)
                s.write(b"M115\n")
                time.sleep(0.8)
                
                resp = s.read(s.in_waiting or 150).decode('utf-8', errors='ignore')
                print(f"[SERIAL] Response read: {resp.strip()}")
                
                if "FIRMWARE_NAME" in resp or "ok" in resp.lower() or "marlin" in resp.lower() or "start" in resp.lower():
                    print(f"[SERIAL] Printer identified on {port}!")
                    serial_port = s
                    printer_port_name = port
                    printer_connected = True
                    break
                
                # Fallback: If it's a USB serial port and opened successfully but timed out,
                # we force connect to it anyway because USB serial converters are almost always the target printer.
                is_usb_serial = False
                for kw in ['usbserial', 'usbmodem', 'ttyusb', 'ttyacm', 'com13', 'com5']:
                    if kw in port.lower():
                        is_usb_serial = True
                        break
                        
                if is_usb_serial:
                    print(f"[SERIAL] Warning: {port} opened successfully but query timed out. Force connecting as fallback...")
                    serial_port = s
                    printer_port_name = port
                    printer_connected = True
                    break
                
                s.close()
            except Exception as e:
                print(f"[SERIAL] Failed on {port} at {baud}: {e}")
                
        if printer_connected:
            # Start reading/writing threads
            threading.Thread(target=printer_reader_thread, daemon=True).start()
            threading.Thread(target=printer_writer_thread, daemon=True).start()
            
            # Startup G-code instructions
            try:
                serial_port.write(b"M999\n") # Reset errors
                time.sleep(0.1)
                serial_port.write(b"G90\n")  # Absolute mode
                time.sleep(0.1)
                serial_port.write(b"G28\n")  # Home
                print(f"[SERIAL] Connected successfully and sent G28 homing to {printer_port_name}.")
            except Exception as ex:
                print(f"[SERIAL ERROR] Failed to send initialization gcode: {ex}")
            return True
            
    print("[SERIAL] Auto-connect failed. No printer detected.")
    return False

def disconnect_printer():
    global serial_port, printer_connected, printer_port_name, in_flight
    if serial_port and serial_port.is_open:
        try:
            serial_port.write(b"M410\n") # Quick stop
            time.sleep(0.1)
            serial_port.close()
        except:
            pass
    serial_port = None
    printer_connected = False
    printer_port_name = None
    in_flight = 0
    print("[SERIAL] Disconnected.")

# --- WebSocket Client Handling ---
def handle_ws_client(conn, addr):
    global serial_queue, printer_connected, printer_port_name
    print(f"[WS] Client connected from {addr}")
    
    # Handshake
    try:
        req = conn.recv(2048).decode('utf-8')
        headers = {}
        for line in req.split("\r\n")[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()
                
        key = headers.get('sec-websocket-key')
        if not key:
            conn.close()
            return
            
        accept = compute_ws_accept(key)
        resp = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
        )
        conn.sendall(resp.encode('utf-8'))
        print("[WS] Handshake complete.")
        
        # Send initial status
        status_msg = f"status:{'connected' if printer_connected else 'disconnected'}:{printer_port_name or ''}"
        conn.sendall(make_ws_frame(status_msg))
    except Exception as e:
        print(f"[WS HANDSHAKE ERROR] {e}")
        conn.close()
        return

    # Receive frames loop
    buffer = bytearray()
    try:
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            buffer.extend(chunk)
            
            while True:
                msg, bytes_used = parse_ws_frame(buffer)
                if msg is None:
                    break
                buffer = buffer[bytes_used:]
                
                # Handle incoming messages
                if msg.startswith("connect"):
                    if not printer_connected:
                        success = try_connect_printer()
                        status = f"status:{'connected' if success else 'disconnected'}:{printer_port_name or ''}"
                        conn.sendall(make_ws_frame(status))
                    else:
                        status = f"status:connected:{printer_port_name}"
                        conn.sendall(make_ws_frame(status))
                        
                elif msg.startswith("disconnect"):
                    disconnect_printer()
                    conn.sendall(make_ws_frame("status:disconnected:"))
                    
                elif msg.startswith("gcode:"):
                    gcode = msg[6:]
                    if printer_connected:
                        if gcode.startswith("G1"):
                            # No queuing: overwrite with the single latest command to prevent lag!
                            with serial_queue_cv:
                                serial_queue = [gcode]
                                serial_queue_cv.notify()
                        else:
                            # Send instant commands (M-codes, G28, etc) straight to port
                            try:
                                print(f"[SERIAL INSTANT] {gcode}")
                                serial_port.write((gcode + "\n").encode('utf-8'))
                            except Exception as ex:
                                print(f"[SERIAL INSTANT ERROR] {ex}")
                                
    except Exception as e:
        print(f"[WS CLIENT ERROR] {e}")
    finally:
        print(f"[WS] Client disconnected from {addr}")
        conn.close()

def start_ws_server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", WS_PORT))
    s.listen(5)
    print(f"WebSocket Server running on port {WS_PORT}")
    
    while keep_running:
        try:
            conn, addr = s.accept()
            threading.Thread(target=handle_ws_client, args=(conn, addr), daemon=True).start()
        except:
            break

# --- Main Boot ---
if __name__ == "__main__":
    print("==================================================")
    print("         NEXUS-4 PRINTER CONTROLLER BACKEND       ")
    print("==================================================")
    
    # Start server threads immediately
    threading.Thread(target=start_http_server, daemon=True).start()
    threading.Thread(target=start_ws_server, daemon=True).start()
    
    # Pre-probe printer connection on startup asynchronously
    threading.Thread(target=try_connect_printer, daemon=True).start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[SERVER] Shutting down...")
        keep_running = False
        disconnect_printer()
        sys.exit(0)
