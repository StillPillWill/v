import { WebSocketServer } from 'ws';

const PORT = 8080;
const wss = new WebSocketServer({ port: PORT });

let visualizer = null;
let controller = null;

wss.on('connection', (ws, req) => {
  const url = req.url || '';
  
  if (url.includes('/visualizer')) {
    if (visualizer) {
      console.log('Replacing existing visualizer connection.');
      visualizer.terminate();
    }
    visualizer = ws;
    console.log('Visualizer (Simulation Client) connected.');
    
    ws.on('close', () => {
      console.log('Visualizer disconnected.');
      visualizer = null;
    });
  } else if (url.includes('/control')) {
    if (controller) {
      console.log('Replacing existing controller connection.');
      controller.terminate();
    }
    controller = ws;
    console.log('External Controller Client (e.g. Python) connected.');
    
    ws.on('close', () => {
      console.log('Controller client disconnected.');
      controller = null;
      // Inform visualizer to fall back to local control mode
      if (visualizer && visualizer.readyState === 1) {
        visualizer.send(JSON.stringify({ type: 'external_disconnect' }));
      }
    });
  } else {
    // Auto-detect based on connection path or default
    console.log(`Unknown connection path: ${url}, defaulting to controller client.`);
    controller = ws;
  }

  // Bidirectional Message Routing
  ws.on('message', (message) => {
    try {
      const dataStr = message.toString();
      
      if (ws === visualizer) {
        // Check for remote browser logs or errors
        if (dataStr.startsWith('{')) {
          try {
            const parsed = JSON.parse(dataStr);
            if (parsed.type === 'browser_log') {
              console.log(`[BROWSER LOG] ${parsed.message}`);
              return;
            } else if (parsed.type === 'browser_error') {
              console.error(`[BROWSER ERROR] ${parsed.message} | Line ${parsed.lineno}`);
              return;
            }
          } catch (_) {}
        }

        // Telemetry Stream: Visualizer -> Controller
        if (controller && controller.readyState === 1) {
          controller.send(dataStr);
        }
      } else if (ws === controller) {
        // Control Target/Torque Commands: Controller -> Visualizer
        if (visualizer && visualizer.readyState === 1) {
          visualizer.send(dataStr);
        }
      }
    } catch (e) {
      console.error('Error bridging socket message:', e);
    }
  });
});

console.log(`Nexus-4 WebSocket API Server listening on ws://localhost:${PORT}`);
console.log('Paths:');
console.log(`  - Visualizer: ws://localhost:${PORT}/visualizer`);
console.log(`  - Control Scripts: ws://localhost:${PORT}/control`);
