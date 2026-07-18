# Nexus-4 Advanced Simulation Controller

A zero-dependency, ultra-low latency Web Serial controller and kinematics simulator for a 4-axis robotic arm (or pen plotter).

## Quick Start (Zero Installation)

Because this application uses the native Web Serial API, it runs entirely in your browser with no backend server, Node.js, or Python dependencies required!

### 1. Clone the repository
```bash
git clone https://github.com/StillPillWill/v.git
cd v
```

### 2. Serve the directory
You just need a basic static file server to serve the HTML/JS files so the browser can load the ES modules. You can use anything:

**Using Python:**
```bash
python -m http.server 8000
```
*Or, if you use VS Code, just click "Go Live" with the Live Server extension.*
*Or using npx: `npx serve .`*

### 3. Connect to your Printer
1. Open Google Chrome or Microsoft Edge and navigate to `http://localhost:8000`
2. Click the **Connect Printer** button in the top right of the dashboard.
3. Your browser will prompt you to select your printer's COM port (e.g. `COM5`).
4. The printer will automatically connect and home itself (`G28`). You're ready to go!

> **Note:** Web Serial requires a Chromium-based browser. Firefox and Safari are not supported.
