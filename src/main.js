import { HandTrackingProcessor } from './handTracker.js';
import { Vec3, AlphaBetaEstimator, MotionShaper } from './webSerialController.js';

// --- Telemetry and Control State ---
let controlMode = 'kinematic'; // 'kinematic', 'planner'
let traceEnabled = true;

// Cartesian targets
let targetX = 292; // Center of 585
let targetY = 387; // Center of 775
let targetZ = 15;

// Raw tracked webcam targets (smoothed continuously in 60Hz loop)
let webcamTargetX = 292;
let webcamTargetY = 387;
let webcamTargetZ = 15;

// Waypoints for Path Planner
let waypoints = [];
let currentWaypointIndex = -1;
let isPlayingPath = false;
let isLoopingPath = false;
let pathProgress = 0;
let pathSpeedScale = 1.0;

// --- WebSocket Connection ---
let socket = null;
let apiConnected = false;
let printerConnected = false;
let printerPortName = "";

// --- Motion Smoothing (AlphaBeta + Shaper) ---
const bedX = 585.0;
const bedY = 775.0;
const estimator = new AlphaBetaEstimator();
const shaper = new MotionShaper(
  200.0,  // MAX_VEL_MM_S
  1500.0, // MAX_ACC_MM_S2
  new Vec3(bedX / 2, bedY / 2, 15.0),
  new Vec3(0, 0, 0),
  new Vec3(bedX, bedY, 100.0)
);

let lastSentX = null;
let lastSentY = null;
let lastSentZ = null;
let lastSentPen = 1.0; // 1.0 = Pen Up (Default)
let targetPen = 1.0;
let lastSentTime = 0;

// --- MediaPipe Webcam Tracking ---
let webcamActive = false;
let mediaPipeCamera = null;
let mediaPipeHands = null;
let fistClosed = false;
const handProcessor = new HandTrackingProcessor();
let lastTrackedTime = 0;

// --- DOM References ---
const dom = {
  tabButtons: document.querySelectorAll('.tab-btn'),
  tabContents: document.querySelectorAll('.tab-content'),
  
  targetXSlider: document.getElementById('target-x'),
  targetYSlider: document.getElementById('target-y'),
  targetZSlider: document.getElementById('target-z'),
  txValText: document.getElementById('tx-val'),
  tyValText: document.getElementById('ty-val'),
  tzValText: document.getElementById('tz-val'),
  
  telemetryX: document.getElementById('telemetry-x'),
  telemetryY: document.getElementById('telemetry-y'),
  telemetryZ: document.getElementById('telemetry-z'),
  telemetryStatus: document.getElementById('telemetry-status'),
  apiStatus: document.getElementById('api-status'),
  btnConnectPrinter: document.getElementById('btn-connect-printer'),
  
  btnAddWaypoint: document.getElementById('btn-add-waypoint'),
  btnClearWaypoints: document.getElementById('btn-clear-waypoints'),
  waypointList: document.getElementById('waypoint-list'),
  btnPlayPath: document.getElementById('btn-play-path'),
  btnLoopPath: document.getElementById('btn-loop-path'),
  pathSpeed: document.getElementById('path-speed'),
  speedVal: document.getElementById('speed-val'),

  toggleWebcam: document.getElementById('toggle-webcam'),
  webcamVideo: document.getElementById('webcam-video'),
  webcamOverlay: document.getElementById('webcam-overlay'),
  trackingStatusText: document.getElementById('tracking-status-text'),
  trackingStatusBadge: document.getElementById('tracking-status-badge'),
  gripStatus: document.getElementById('grip-status')
};

// --- WebSocket Management ---
function connectWebSocket() {
  console.log("[WS] Connecting to server...");
  socket = new WebSocket("ws://localhost:8765");

  socket.onopen = () => {
    console.log("[WS] Connected to server.");
    apiConnected = true;
    dom.apiStatus.textContent = 'SERVER OK';
    dom.apiStatus.className = 'value status-badge online';
  };

  socket.onmessage = (event) => {
    const data = event.data;
    if (data.startsWith("status:")) {
      const parts = data.split(":");
      const status = parts[1];
      const port = parts[2] || "";
      
      if (status === "connected") {
        printerConnected = true;
        printerPortName = port;
        dom.btnConnectPrinter.textContent = `Disconnect (${port})`;
        dom.btnConnectPrinter.classList.remove('btn-primary');
        dom.btnConnectPrinter.classList.add('btn-danger');
      } else {
        printerConnected = false;
        printerPortName = "";
        dom.btnConnectPrinter.textContent = 'Connect Printer';
        dom.btnConnectPrinter.classList.remove('btn-danger');
        dom.btnConnectPrinter.classList.add('btn-primary');
      }
    }
  };

  socket.onclose = () => {
    console.log("[WS] Connection lost. Reconnecting in 2s...");
    apiConnected = false;
    printerConnected = false;
    dom.apiStatus.textContent = 'SERVER OFFLINE';
    dom.apiStatus.className = 'value status-badge offline';
    dom.btnConnectPrinter.textContent = 'Connect Printer';
    dom.btnConnectPrinter.classList.remove('btn-danger');
    dom.btnConnectPrinter.classList.add('btn-primary');
    setTimeout(connectWebSocket, 2000);
  };
}

// --- Initialize Event Handlers ---
function initEvents() {
  // Tabs Switcher
  dom.tabButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      dom.tabButtons.forEach(b => b.classList.remove('active'));
      dom.tabContents.forEach(c => c.classList.remove('active'));
      
      btn.classList.add('active');
      const tabId = `tab-${btn.dataset.tab}`;
      document.getElementById(tabId).classList.add('active');
      controlMode = btn.dataset.tab;
      
      if (controlMode === 'kinematic') {
        dom.targetXSlider.value = Math.round(targetX);
        dom.targetYSlider.value = Math.round(targetY);
        dom.targetZSlider.value = Math.round(targetZ);
        updateCartesianTexts();
      }
    });
  });

  // Connect Printer click handler
  dom.btnConnectPrinter.addEventListener('click', () => {
    if (!apiConnected) {
      alert("Server is offline. Start 'python server.py' first!");
      return;
    }
    if (printerConnected) {
      socket.send("disconnect");
    } else {
      socket.send("connect");
    }
  });

  // Cartesian Slider Events
  const onCartesianInput = () => {
    targetX = parseInt(dom.targetXSlider.value);
    targetY = parseInt(dom.targetYSlider.value);
    targetZ = parseInt(dom.targetZSlider.value);
    
    updateCartesianTexts();
    
    if (isPlayingPath) pausePathPlayback();
    if (webcamActive) {
      dom.toggleWebcam.checked = false;
      disableWebcamTracking();
    }
  };

  dom.targetXSlider.addEventListener('input', onCartesianInput);
  dom.targetYSlider.addEventListener('input', onCartesianInput);
  dom.targetZSlider.addEventListener('input', onCartesianInput);

  dom.pathSpeed.addEventListener('input', (e) => {
    pathSpeedScale = parseFloat(e.target.value);
    dom.speedVal.textContent = e.target.value;
  });

  // Waypoints Handlers
  dom.btnAddWaypoint.addEventListener('click', () => {
    waypoints.push({ x: targetX, y: targetY, z: targetZ });
    renderWaypointsList();
  });

  dom.btnClearWaypoints.addEventListener('click', () => {
    waypoints = [];
    renderWaypointsList();
    pausePathPlayback();
  });

  dom.btnPlayPath.addEventListener('click', () => {
    if (waypoints.length < 2) {
      alert('Add at least 2 waypoints to play a path!');
      return;
    }
    if (isPlayingPath) {
      pausePathPlayback();
    } else {
      startPathPlayback();
    }
  });

  dom.btnLoopPath.addEventListener('click', () => {
    isLoopingPath = !isLoopingPath;
    dom.btnLoopPath.textContent = `Loop: ${isLoopingPath ? 'ON' : 'OFF'}`;
    dom.btnLoopPath.classList.toggle('btn-primary', isLoopingPath);
  });

  // Webcam Tracking Toggle
  dom.toggleWebcam.addEventListener('change', (e) => {
    if (e.target.checked) {
      enableWebcamTracking();
    } else {
      disableWebcamTracking();
    }
  });

}

function updateCartesianTexts() {
  dom.txValText.textContent = Math.round(targetX);
  dom.tyValText.textContent = Math.round(targetY);
  dom.tzValText.textContent = Math.round(targetZ);
}

// --- Waypoints rendering ---
function renderWaypointsList() {
  dom.waypointList.innerHTML = '';
  if (waypoints.length === 0) {
    dom.waypointList.innerHTML = '<li class="empty-list-msg">No waypoints defined yet. Add current position.</li>';
    return;
  }

  waypoints.forEach((wp, idx) => {
    const li = document.createElement('li');
    if (idx === currentWaypointIndex) {
      li.classList.add('active');
    }
    
    li.innerHTML = `
      <div>
        <span class="wp-num">WP #${idx + 1}</span>
        <span class="wp-coords mono">(${Math.round(wp.x)}, ${Math.round(wp.y)}, ${Math.round(wp.z)})</span>
      </div>
      <button class="btn-remove" data-index="${idx}">×</button>
    `;

    li.addEventListener('click', (e) => {
      if (e.target.classList.contains('btn-remove')) {
        e.stopPropagation();
        const removeIdx = parseInt(e.target.dataset.index);
        waypoints.splice(removeIdx, 1);
        if (currentWaypointIndex >= waypoints.length) {
          currentWaypointIndex = waypoints.length - 1;
        }
        renderWaypointsList();
        if (isPlayingPath) pausePathPlayback();
        return;
      }
      
      currentWaypointIndex = idx;
      renderWaypointsList();
      targetX = wp.x;
      targetY = wp.y;
      targetZ = wp.z;
      
      dom.targetXSlider.value = Math.round(targetX);
      dom.targetYSlider.value = Math.round(targetY);
      dom.targetZSlider.value = Math.round(targetZ);
      updateCartesianTexts();
    });

    dom.waypointList.appendChild(li);
  });
}

function startPathPlayback() {
  isPlayingPath = true;
  dom.btnPlayPath.textContent = 'Pause Path';
  dom.btnPlayPath.classList.remove('btn-success');
  dom.btnPlayPath.classList.add('btn-danger');
  pathProgress = 0;
  if (currentWaypointIndex < 0 || currentWaypointIndex >= waypoints.length - 1) {
    currentWaypointIndex = 0;
  }
  renderWaypointsList();
}

function pausePathPlayback() {
  isPlayingPath = false;
  dom.btnPlayPath.textContent = 'Play Path';
  dom.btnPlayPath.classList.remove('btn-danger');
  dom.btnPlayPath.classList.add('btn-success');
}

// --- MediaPipe Hand Tracking Setup ---
function enableWebcamTracking() {
  if (webcamActive) return;

  dom.trackingStatusText.textContent = 'INITIALIZING...';
  dom.trackingStatusText.className = 'tracking-status lost';
  dom.trackingStatusBadge.textContent = 'INITIALIZING';
  dom.trackingStatusBadge.className = 'status-badge offline';

  try {
    if (!mediaPipeHands) {
      mediaPipeHands = new window.Hands({
        locateFile: (file) => `https://cdn.jsdelivr.net/npm/@mediapipe/hands/${file}`
      });

      mediaPipeHands.setOptions({
        maxNumHands: 1,
        modelComplexity: 0, // Light/fastest model
        minDetectionConfidence: 0.5,
        minTrackingConfidence: 0.5
      });

      mediaPipeHands.onResults(onHandResults);
    }

    navigator.mediaDevices.getUserMedia({ video: { width: 640, height: 480 } })
      .then((stream) => {
        dom.webcamVideo.srcObject = stream;
        webcamActive = true;
        
        mediaPipeCamera = new window.Camera(dom.webcamVideo, {
          onFrame: async () => {
            if (webcamActive) {
              const canvas = dom.webcamOverlay;
              const ctx = canvas.getContext('2d');
              if (ctx) ctx.clearRect(0, 0, canvas.width, canvas.height);

              await mediaPipeHands.send({ image: dom.webcamVideo });
            }
          },
          width: 640,
          height: 480
        });
        
        mediaPipeCamera.start();
      })
      .catch((err) => {
        console.error('Error starting camera stream:', err);
        alert('Webcam permission denied or camera not found.');
        dom.toggleWebcam.checked = false;
        disableWebcamTracking();
      });
  } catch (err) {
    console.error('MediaPipe initialization error:', err);
    alert('Failed to load MediaPipe tracking libraries.');
    dom.toggleWebcam.checked = false;
    disableWebcamTracking();
  }
}

function disableWebcamTracking() {
  webcamActive = false;
  
  if (mediaPipeCamera) {
    mediaPipeCamera.stop();
    mediaPipeCamera = null;
  }
  
  if (dom.webcamVideo.srcObject) {
    dom.webcamVideo.srcObject.getTracks().forEach(track => track.stop());
    dom.webcamVideo.srcObject = null;
  }

  const canvas = dom.webcamOverlay;
  const ctx = canvas.getContext('2d');
  if (ctx) ctx.clearRect(0, 0, canvas.width, canvas.height);

  handProcessor.reset();
  
  dom.trackingStatusText.textContent = 'TRACKING OFF';
  dom.trackingStatusText.className = 'tracking-status lost';
  dom.trackingStatusBadge.textContent = 'TRACKING OFF';
  dom.trackingStatusBadge.className = 'status-badge offline';

  fistClosed = false;
  dom.gripStatus.textContent = 'OPEN (PEN UP)';
  dom.gripStatus.className = 'status-badge offline';
  dom.gripStatus.style.background = 'rgba(139, 148, 158, 0.15)';
  dom.gripStatus.style.color = '#8b949e';
  dom.gripStatus.style.borderColor = 'rgba(139, 148, 158, 0.3)';
}

function onHandResults(results) {
  if (!webcamActive) return;

  const canvas = dom.webcamOverlay;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;

  const video = dom.webcamVideo;
  if (video.videoWidth > 0 && video.videoHeight > 0) {
    if (canvas.width !== video.videoWidth || canvas.height !== video.videoHeight) {
      canvas.width = video.videoWidth;
      canvas.height = video.videoHeight;
    }
  } else {
    return;
  }

  const timestamp = performance.now();
  if (!handProcessor.isTracking || lastTrackedTime === 0) {
    lastTrackedTime = timestamp;
  }
  const dt = (timestamp - lastTrackedTime) / 1000;
  lastTrackedTime = timestamp;

  if (results.multiHandLandmarks && results.multiHandLandmarks.length > 0) {
    dom.trackingStatusText.textContent = 'ACTIVE';
    dom.trackingStatusText.className = 'tracking-status active';
    dom.trackingStatusBadge.textContent = 'ACTIVE';
    dom.trackingStatusBadge.className = 'status-badge online';

    const landmarks = results.multiHandLandmarks[0];
    fistClosed = checkFistClosed(landmarks);

    if (fistClosed) {
      dom.gripStatus.textContent = 'CLOSED (PEN DOWN)';
      dom.gripStatus.className = 'status-badge';
      dom.gripStatus.style.background = 'rgba(255, 170, 0, 0.15)';
      dom.gripStatus.style.color = '#ffaa00';
      dom.gripStatus.style.borderColor = 'rgba(255, 170, 0, 0.3)';
      targetPen = 0.0;
    } else {
      dom.gripStatus.textContent = 'OPEN (PEN UP)';
      dom.gripStatus.className = 'status-badge offline';
      dom.gripStatus.style.background = 'rgba(139, 148, 158, 0.15)';
      dom.gripStatus.style.color = '#8b949e';
      dom.gripStatus.style.borderColor = 'rgba(139, 148, 158, 0.3)';
      targetPen = 1.0;
    }

    drawHandSkeleton(ctx, landmarks);

    const targets = handProcessor.processFrame(landmarks, dt);
    if (targets) {
      webcamTargetX = targets.x;
      webcamTargetY = targets.y;
      webcamTargetZ = fistClosed ? 0.0 : 15.0; // Force Pen Down/Up heights
    }
  } else {
    dom.trackingStatusText.textContent = 'LOST';
    dom.trackingStatusText.className = 'tracking-status lost';
    dom.trackingStatusBadge.textContent = 'LOST';
    dom.trackingStatusBadge.className = 'status-badge offline';
    
    handProcessor.isTracking = false;
    fistClosed = false;
    targetPen = 1.0;

    dom.gripStatus.textContent = 'OPEN (PEN UP)';
    dom.gripStatus.className = 'status-badge offline';
    dom.gripStatus.style.background = 'rgba(139, 148, 158, 0.15)';
    dom.gripStatus.style.color = '#8b949e';
    dom.gripStatus.style.borderColor = 'rgba(139, 148, 158, 0.3)';
  }
}

function checkFistClosed(landmarks) {
  const dx = landmarks[9].x - landmarks[0].x;
  const dy = landmarks[9].y - landmarks[0].y;
  const dz = landmarks[9].z - landmarks[0].z;
  const handSize = Math.sqrt(dx*dx + dy*dy + dz*dz) + 1e-6;

  const indexDist = Math.sqrt((landmarks[8].x - landmarks[5].x)**2 + (landmarks[8].y - landmarks[5].y)**2 + (landmarks[8].z - landmarks[5].z)**2) / handSize;
  const middleDist = Math.sqrt((landmarks[12].x - landmarks[9].x)**2 + (landmarks[12].y - landmarks[9].y)**2 + (landmarks[12].z - landmarks[9].z)**2) / handSize;
  const ringDist = Math.sqrt((landmarks[16].x - landmarks[13].x)**2 + (landmarks[16].y - landmarks[13].y)**2 + (landmarks[16].z - landmarks[13].z)**2) / handSize;
  const pinkyDist = Math.sqrt((landmarks[20].x - landmarks[17].x)**2 + (landmarks[20].y - landmarks[17].y)**2 + (landmarks[20].z - landmarks[17].z)**2) / handSize;

  return indexDist < 0.50 && middleDist < 0.50 && ringDist < 0.50 && pinkyDist < 0.50;
}

function drawHandSkeleton(ctx, landmarks) {
  const width = ctx.canvas.width;
  const height = ctx.canvas.height;

  ctx.strokeStyle = fistClosed ? '#ffaa00' : '#00f0ff';
  ctx.lineWidth = 3;
  ctx.shadowBlur = 4;
  ctx.shadowColor = fistClosed ? '#ffaa00' : '#00f0ff';

  const connections = [
    [0, 1], [1, 2], [2, 3], [3, 4],
    [0, 5], [5, 6], [6, 7], [7, 8],
    [0, 9], [9, 10], [10, 11], [11, 12],
    [0, 13], [13, 14], [14, 15], [15, 16],
    [0, 17], [17, 18], [18, 19], [19, 20],
    [5, 9], [9, 13], [13, 17]
  ];

  connections.forEach(([i1, i2]) => {
    const pt1 = landmarks[i1];
    const pt2 = landmarks[i2];
    ctx.beginPath();
    ctx.moveTo(pt1.x * width, pt1.y * height);
    ctx.lineTo(pt2.x * width, pt2.y * height);
    ctx.stroke();
  });

  ctx.fillStyle = fistClosed ? '#ffaa00' : '#ff007f';
  ctx.shadowColor = fistClosed ? '#ffaa00' : '#ff007f';
  landmarks.forEach((pt) => {
    ctx.beginPath();
    ctx.arc(pt.x * width, pt.y * height, 5, 0, 2 * Math.PI);
    ctx.fill();
  });

  ctx.shadowBlur = 0;
}

// --- Main Loop (60Hz coordinate smoothing & server streaming) ---
let lastTime = 0;

function update(time) {
  requestAnimationFrame(update);

  if (lastTime === 0) {
    lastTime = time;
    return;
  }

  let dt = (time - lastTime) / 1000;
  if (dt < 0.001) return;
  if (dt > 0.1) dt = 0.1;
  lastTime = time;

  // 1. Path Planner Execution
  if (isPlayingPath && waypoints.length >= 2) {
    pathProgress += dt * 0.25 * pathSpeedScale;
    
    if (pathProgress >= 1.0) {
      pathProgress = 0;
      currentWaypointIndex++;
      
      if (currentWaypointIndex >= waypoints.length - 1) {
        if (isLoopingPath) {
          currentWaypointIndex = 0;
        } else {
          currentWaypointIndex = waypoints.length - 1;
          pausePathPlayback();
        }
      }
      renderWaypointsList();
    }

    if (isPlayingPath) {
      const startWp = waypoints[currentWaypointIndex];
      const endWp = waypoints[currentWaypointIndex + 1];
      
      if (startWp && endWp) {
        targetX = startWp.x + (endWp.x - startWp.x) * pathProgress;
        targetY = startWp.y + (endWp.y - startWp.y) * pathProgress;
        targetZ = startWp.z + (endWp.z - startWp.z) * pathProgress;

        dom.targetXSlider.value = Math.round(targetX);
        dom.targetYSlider.value = Math.round(targetY);
        dom.targetZSlider.value = Math.round(targetZ);
        updateCartesianTexts();
      }
    }
  }

  // 2. Webcam Coordinate Smoothing
  if (webcamActive && handProcessor.isTracking) {
    const k_pos = 12.0;
    
    targetX += (webcamTargetX - targetX) * (1 - Math.exp(-k_pos * dt));
    targetY += (webcamTargetY - targetY) * (1 - Math.exp(-k_pos * dt));
    targetZ += (webcamTargetZ - targetZ) * (1 - Math.exp(-k_pos * dt));
    
    updateCartesianTexts();
    
    dom.targetXSlider.value = Math.round(targetX);
    dom.targetYSlider.value = Math.round(targetY);
    dom.targetZSlider.value = Math.round(targetZ);
  }

  // Update UI Telemetry Headers
  dom.telemetryX.textContent = targetX.toFixed(1);
  dom.telemetryY.textContent = targetY.toFixed(1);
  dom.telemetryZ.textContent = targetZ.toFixed(1);

  // Set Status Badge in UI
  const telemetryStatus = dom.telemetryStatus;
  if (!apiConnected) {
    telemetryStatus.textContent = 'SERVER DISCONNECTED';
    telemetryStatus.className = 'value status-badge offline';
  } else if (!printerConnected) {
    telemetryStatus.textContent = 'PRINTER OFFLINE';
    telemetryStatus.className = 'value status-badge offline';
  } else if (webcamActive && handProcessor.isTracking) {
    telemetryStatus.textContent = 'WEBCAM DRIVING';
    telemetryStatus.style.background = 'rgba(57, 255, 20, 0.15)';
    telemetryStatus.style.color = '#39ff14';
    telemetryStatus.style.borderColor = 'rgba(57, 255, 20, 0.3)';
  } else if (isPlayingPath) {
    telemetryStatus.textContent = 'RUNNING PLANNER';
    telemetryStatus.style.background = 'rgba(0, 240, 255, 0.15)';
    telemetryStatus.style.color = '#00f0ff';
    telemetryStatus.style.borderColor = 'rgba(0, 240, 255, 0.3)';
  } else {
    telemetryStatus.textContent = 'STANDBY';
    telemetryStatus.style.background = 'rgba(16, 185, 129, 0.15)';
    telemetryStatus.style.color = '#10b981';
    telemetryStatus.style.borderColor = 'rgba(16, 185, 129, 0.3)';
  }

  // 3. Motion Interpolation & Dynamic G-code streaming
  if (apiConnected && printerConnected) {
    const nowSec = performance.now() / 1000.0;
    
    // Feed estimator & shaper
    estimator.update(new Vec3(targetX, targetY, targetZ), nowSec);
    const estimated = estimator.predict(nowSec);
    const safePos = shaper.step(estimated, dt);

    if (lastSentX === null || lastSentY === null || lastSentZ === null) {
      lastSentX = safePos.x;
      lastSentY = safePos.y;
      lastSentZ = safePos.z;
    }

    const dx = safePos.x - lastSentX;
    const dy = safePos.y - lastSentY;
    const dz = safePos.z - lastSentZ;
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    
    const nowMs = performance.now();

    // Limit movement G-code frequency to 50Hz (20ms interval) to keep queue clean
    if (dist > 0.05 && (nowMs - lastSentTime > 20)) {
      const vNorm = shaper.vel.norm();
      const feedrate = Math.floor(Math.min(99999, Math.max(1000, vNorm * 60)));
      
      socket.send(`gcode:G1 X${safePos.x.toFixed(1)} Y${safePos.y.toFixed(1)} Z${safePos.z.toFixed(1)} F${feedrate}`);
      
      lastSentX = safePos.x;
      lastSentY = safePos.y;
      lastSentZ = safePos.z;
      lastSentTime = nowMs;
    }
  }
}

// --- Initialize App ---
initEvents();
connectWebSocket();
requestAnimationFrame(update);
console.log("Nexus-4 Printer Controller Initialized.");
