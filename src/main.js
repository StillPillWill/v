import { HandTrackingProcessor } from './handTracker.js';
import { Vec3, AlphaBetaEstimator, MotionShaper } from './webSerialController.js';
import { ImagePlotter, GenerativePlotter } from './plotter.js';

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

let lastSentX = null;
let lastSentY = null;
let lastSentZ = null;
let lastSentTime = 0;

// --- MediaPipe Webcam Tracking ---
let webcamActive = false;
let mediaPipeCamera = null;
let mediaPipeHands = null;
let fistClosed = false;
let fistConfidence = 0.0;
let penDownZ = 0.0;
let penUpZ = 15.0;
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
  gripStatus: document.getElementById('grip-status'),
  penDownZSlider: document.getElementById('pen-down-z'),
  penUpZSlider: document.getElementById('pen-up-z'),
  workspaceWidthInput: document.getElementById('workspace-width'),
  workspaceHeightInput: document.getElementById('workspace-height'),
  // Plotter
  plotterDropzone:   document.getElementById('plotter-dropzone'),
  plotterFileInput:  document.getElementById('plotter-file-input'),
  plotterBtnFile:    document.getElementById('btn-plotter-file'),
  plotterBtnCamera:  document.getElementById('btn-plotter-camera'),
  plotterCameraArea: document.getElementById('plotter-camera-area'),
  plotterCamVideo:   document.getElementById('plotter-cam-video'),
  plotterBtnSnap:    document.getElementById('btn-plotter-snap'),
  plotterPreview:    document.getElementById('plotter-preview'),
  plotterLines:      document.getElementById('plotter-lines'),
  plotterThreshold:  document.getElementById('plotter-threshold'),
  plotterMinStroke:  document.getElementById('plotter-min-stroke'),
  plotterSimplification: document.getElementById('plotter-simplification'),
  plotterMergeGap:   document.getElementById('plotter-merge-gap'),
  plotterMode:       document.getElementById('plotter-mode'),
  plotterLineWidth:  document.getElementById('plotter-line-width'),
  plotterStyle:      document.getElementById('plotter-style'),
  plotterShadingDensity: document.getElementById('plotter-shading-density'),
  plotterStatus:     document.getElementById('plotter-status'),
  plotterBtnProcess: document.getElementById('btn-plotter-process'),
  plotterBtnPlot:    document.getElementById('btn-plotter-plot'),
  plotterBtnPause:   document.getElementById('btn-plotter-pause'),
  plotterBtnStop:    document.getElementById('btn-plotter-stop'),
  plotterActiveControls: document.getElementById('plotter-active-controls'),
  plotterProgressContainer: document.getElementById('plotter-progress-container'),
  plotterProgressPct: document.getElementById('plotter-progress-pct'),
  plotterProgressEta: document.getElementById('plotter-progress-eta'),
  plotterProgressBar: document.getElementById('plotter-progress-bar'),
  plotterProgressCounts: document.getElementById('plotter-progress-counts'),

  // Generative Art
  generativePreview: document.getElementById('generative-preview'),
  genPatternType: document.getElementById('gen-pattern-type'),
  genP1: document.getElementById('gen-p1'),
  genP2: document.getElementById('gen-p2'),
  lblGenP1: document.getElementById('lbl-gen-p1'),
  lblGenP2: document.getElementById('lbl-gen-p2'),
  generativeStatus: document.getElementById('generative-status'),
  btnGenerativeGenerate: document.getElementById('btn-generative-generate'),
  btnGenerativePlot: document.getElementById('btn-generative-plot'),
  generativeActiveControls: document.getElementById('generative-active-controls'),
  btnGenerativePause: document.getElementById('btn-generative-pause'),
  btnGenerativeStop: document.getElementById('btn-generative-stop'),
  generativeProgressContainer: document.getElementById('generative-progress-container'),
  generativeProgressPct: document.getElementById('generative-progress-pct'),
  generativeProgressEta: document.getElementById('generative-progress-eta'),
  generativeProgressBar: document.getElementById('generative-progress-bar'),
  generativeProgressCounts: document.getElementById('generative-progress-counts')
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
    } else if (data === "gcode-ok") {
      if (imagePlotter && imagePlotter._pendingOkResolvers && imagePlotter._pendingOkResolvers.length > 0) {
        const resolve = imagePlotter._pendingOkResolvers.shift();
        if (resolve) resolve();
      }
      if (generativePlotter && generativePlotter._pendingOkResolvers && generativePlotter._pendingOkResolvers.length > 0) {
        const resolve = generativePlotter._pendingOkResolvers.shift();
        if (resolve) resolve();
      }
    } else if (data.startsWith("ml-result:")) {
      const b64Data = data.slice(10);
      setPlotterStatus('AI Subject Extracted! Generating vector paths…');
      imagePlotter.loadFromDataUrl(b64Data).then(() => {
        const numLines  = parseInt(dom.plotterLines.value, 10)    || 160;
        const threshold = parseInt(dom.plotterThreshold.value, 10) || 30;
        const minStroke = parseInt(dom.plotterMinStroke?.value, 10) || 8;
        const simplification = parseFloat(dom.plotterSimplification?.value) || 1.5;
        const mergeGapMM = parseFloat(dom.plotterMergeGap?.value) || 4.0;
        const lineWidthMM = parseFloat(dom.plotterLineWidth?.value) || 0.8;
        const drawingStyle = dom.plotterStyle?.value || 'outlines';
        const shadingDensity = parseInt(dom.plotterShadingDensity?.value, 10) || 6;

        const wW = parseFloat(dom.workspaceWidthInput?.value)  || handProcessor.workspaceWidth  || 200;
        const wH = parseFloat(dom.workspaceHeightInput?.value) || handProcessor.workspaceHeight || 200;

        const workspace = {
          centerX: handProcessor.centerX,
          centerY: handProcessor.centerY,
          workspaceWidth:  wW,
          workspaceHeight: wH
        };

        const count = imagePlotter.process(
          numLines, threshold, minStroke, simplification, mergeGapMM, 'lineart', lineWidthMM, drawingStyle, shadingDensity, workspace, penDownZ, penUpZ
        );

        dom.plotterPreview.style.display = 'block';
        dom.plotterBtnPlot.disabled = false;
        setPlotterStatus(`AI Subject Line Art Ready — ${count} moves! Hit "Start Plotting".`);
      }).catch(err => {
        setPlotterStatus('AI processing error: ' + err.message);
      });
    } else if (data.startsWith("ml-error:")) {
      setPlotterStatus('AI Error: ' + data.slice(9));
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

  // Initialize values from HTML inputs on load
  if (dom.penDownZSlider) penDownZ = parseFloat(dom.penDownZSlider.value) || 0.0;
  if (dom.penUpZSlider) penUpZ = parseFloat(dom.penUpZSlider.value) || 15.0;
  if (dom.workspaceWidthInput) handProcessor.workspaceWidth = parseFloat(dom.workspaceWidthInput.value) || 200.0;
  if (dom.workspaceHeightInput) handProcessor.workspaceHeight = parseFloat(dom.workspaceHeightInput.value) || 200.0;

  // Pen Height calibration number inputs (updates live, listening to both input and change events for cross-browser safety)
  const handlePenDownChange = (e) => {
    const val = parseFloat(e.target.value);
    if (!isNaN(val)) {
      penDownZ = val;
      console.log("[CALIBRATION] Pen Down Z updated to:", penDownZ);
    }
  };
  dom.penDownZSlider.addEventListener('input', handlePenDownChange);
  dom.penDownZSlider.addEventListener('change', handlePenDownChange);

  const handlePenUpChange = (e) => {
    const val = parseFloat(e.target.value);
    if (!isNaN(val)) {
      penUpZ = val;
      console.log("[CALIBRATION] Pen Up Z updated to:", penUpZ);
    }
  };
  dom.penUpZSlider.addEventListener('input', handlePenUpChange);
  dom.penUpZSlider.addEventListener('change', handlePenUpChange);

  // Configurable Workspace Width handler
  const handleWorkspaceWidthChange = (e) => {
    const val = parseFloat(e.target.value);
    if (!isNaN(val) && val >= 50.0 && val <= 500.0) {
      handProcessor.workspaceWidth = val;
      console.log("[CALIBRATION] Workspace Width X updated to:", val, "mm");
    }
  };
  dom.workspaceWidthInput.addEventListener('input', handleWorkspaceWidthChange);
  dom.workspaceWidthInput.addEventListener('change', handleWorkspaceWidthChange);

  // Configurable Workspace Height handler
  const handleWorkspaceHeightChange = (e) => {
    const val = parseFloat(e.target.value);
    if (!isNaN(val) && val >= 50.0 && val <= 600.0) {
      handProcessor.workspaceHeight = val;
      console.log("[CALIBRATION] Workspace Height Y updated to:", val, "mm");
    }
  };
  dom.workspaceHeightInput.addEventListener('input', handleWorkspaceHeightChange);
  dom.workspaceHeightInput.addEventListener('change', handleWorkspaceHeightChange);
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
    const instantFist = checkFistClosed(landmarks);

    // Apply fist hysteresis confidence filter to prevent single-frame drops
    if (instantFist) {
      // Rapid charge (takes ~2 frames to register fist)
      fistConfidence = Math.min(1.0, fistConfidence + 0.5);
    } else {
      // Slower decay (takes ~12 frames to release pen, bridging tracking drops!)
      fistConfidence = Math.max(0.0, fistConfidence - 0.08);
    }
    fistClosed = (fistConfidence > 0.4);

    if (fistClosed) {
      dom.gripStatus.textContent = 'CLOSED (PEN DOWN)';
      dom.gripStatus.className = 'status-badge';
      dom.gripStatus.style.background = 'rgba(255, 170, 0, 0.15)';
      dom.gripStatus.style.color = '#ffaa00';
      dom.gripStatus.style.borderColor = 'rgba(255, 170, 0, 0.3)';
    } else {
      dom.gripStatus.textContent = 'OPEN (PEN UP)';
      dom.gripStatus.className = 'status-badge offline';
      dom.gripStatus.style.background = 'rgba(139, 148, 158, 0.15)';
      dom.gripStatus.style.color = '#8b949e';
      dom.gripStatus.style.borderColor = 'rgba(139, 148, 158, 0.3)';
    }

    drawHandSkeleton(ctx, landmarks);

    const targets = handProcessor.processFrame(landmarks, dt);
    if (targets) {
      webcamTargetX = targets.x;
      webcamTargetY = targets.y;
      webcamTargetZ = fistClosed ? penDownZ : penUpZ; // Use adjustable calibrated heights!
    }
  } else {
    dom.trackingStatusText.textContent = 'LOST';
    dom.trackingStatusText.className = 'tracking-status lost';
    dom.trackingStatusBadge.textContent = 'LOST';
    dom.trackingStatusBadge.className = 'status-badge offline';
    
    handProcessor.isTracking = false;

    // Do NOT reset fistClosed, fistConfidence, or Z target when tracking is lost!
    // They freeze at their last valid values so the pen doesn't jump up when the hand goes off-screen.
    if (fistClosed) {
      dom.gripStatus.textContent = 'CLOSED (PEN DOWN) - FROZEN';
      dom.gripStatus.className = 'status-badge';
      dom.gripStatus.style.background = 'rgba(255, 170, 0, 0.15)';
      dom.gripStatus.style.color = '#ffaa00';
      dom.gripStatus.style.borderColor = 'rgba(255, 170, 0, 0.3)';
    } else {
      dom.gripStatus.textContent = 'OPEN (PEN UP) - FROZEN';
      dom.gripStatus.className = 'status-badge offline';
      dom.gripStatus.style.background = 'rgba(139, 148, 158, 0.15)';
      dom.gripStatus.style.color = '#8b949e';
      dom.gripStatus.style.borderColor = 'rgba(139, 148, 158, 0.3)';
    }
  }
}

function checkFistClosed(landmarks) {
  // Distance from wrist (0) to knuckle (PIP joints: 6, 10, 14, 18)
  const d0_6 = Math.hypot(landmarks[6].x - landmarks[0].x, landmarks[6].y - landmarks[0].y, landmarks[6].z - landmarks[0].z);
  const d0_8 = Math.hypot(landmarks[8].x - landmarks[0].x, landmarks[8].y - landmarks[0].y, landmarks[8].z - landmarks[0].z);

  const d0_10 = Math.hypot(landmarks[10].x - landmarks[0].x, landmarks[10].y - landmarks[0].y, landmarks[10].z - landmarks[0].z);
  const d0_12 = Math.hypot(landmarks[12].x - landmarks[0].x, landmarks[12].y - landmarks[0].y, landmarks[12].z - landmarks[0].z);

  const d0_14 = Math.hypot(landmarks[14].x - landmarks[0].x, landmarks[14].y - landmarks[0].y, landmarks[14].z - landmarks[0].z);
  const d0_16 = Math.hypot(landmarks[16].x - landmarks[0].x, landmarks[16].y - landmarks[0].y, landmarks[16].z - landmarks[0].z);

  const d0_18 = Math.hypot(landmarks[18].x - landmarks[0].x, landmarks[18].y - landmarks[0].y, landmarks[18].z - landmarks[0].z);
  const d0_20 = Math.hypot(landmarks[20].x - landmarks[0].x, landmarks[20].y - landmarks[0].y, landmarks[20].z - landmarks[0].z);

  // A finger is folded if its tip is closer to the wrist than its PIP joint
  const indexFolded = d0_8 < d0_6;
  const middleFolded = d0_12 < d0_10;
  const ringFolded = d0_16 < d0_14;
  const pinkyFolded = d0_20 < d0_18;

  // We require at least 3 out of 4 fingers to be folded to call it a fist (discounts thumb)
  const foldedCount = (indexFolded ? 1 : 0) + (middleFolded ? 1 : 0) + (ringFolded ? 1 : 0) + (pinkyFolded ? 1 : 0);
  return foldedCount >= 3;
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
    // Distance between current target and new tracked target
    const dx = webcamTargetX - targetX;
    const dy = webcamTargetY - targetY;
    const dist = Math.sqrt(dx * dx + dy * dy);
    
    // Dynamic smoothing factor based on target distance (adaptive jitter filter)
    let k_pos = 4.5;
    if (dist < 1.5) {
      // Micro-tremors / static hand jitter (under 1.5mm deviation): freeze movement almost completely
      k_pos = 0.25;
    } else if (dist < 6.0) {
      // Precision drawing speeds: moderate smoothing
      k_pos = 1.0 + (dist - 1.5) * (3.5 / 4.5); // interpolates smoothly from 1.0 to 4.5
    } else {
      // Quick travel / strokes (intentional movement): high responsiveness
      k_pos = 6.5;
    }
    
    targetX += (webcamTargetX - targetX) * (1 - Math.exp(-k_pos * dt));
    targetY += (webcamTargetY - targetY) * (1 - Math.exp(-k_pos * dt));
    targetZ = webcamTargetZ; // Instantly go to Pen Down/Up heights
    
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
    telemetryStatus.className = 'value status-badge online';
    telemetryStatus.style.background = 'rgba(57, 255, 20, 0.15)';
    telemetryStatus.style.color = '#39ff14';
    telemetryStatus.style.borderColor = 'rgba(57, 255, 20, 0.3)';
  } else if (isPlayingPath) {
    telemetryStatus.textContent = 'RUNNING PLANNER';
    telemetryStatus.className = 'value status-badge online';
    telemetryStatus.style.background = 'rgba(0, 240, 255, 0.15)';
    telemetryStatus.style.color = '#00f0ff';
    telemetryStatus.style.borderColor = 'rgba(0, 240, 255, 0.3)';
  } else {
    telemetryStatus.textContent = 'STANDBY';
    telemetryStatus.className = 'value status-badge';
    telemetryStatus.style.background = 'rgba(16, 185, 129, 0.15)';
    telemetryStatus.style.color = '#10b981';
    telemetryStatus.style.borderColor = 'rgba(16, 185, 129, 0.3)';
  }

  // 3. Dynamic G-code streaming
  if (apiConnected && printerConnected) {
    if (lastSentX === null || lastSentY === null || lastSentZ === null) {
      lastSentX = targetX;
      lastSentY = targetY;
      lastSentZ = targetZ;
    }

    const dx = targetX - lastSentX;
    const dy = targetY - lastSentY;
    const dz = targetZ - lastSentZ;
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    
    const nowMs = performance.now();

    // Limit movement G-code frequency to 50Hz (20ms interval) to keep queue clean
    if (dist > 0.05 && (nowMs - lastSentTime > 20)) {
      socket.send(`gcode:G1 X${targetX.toFixed(1)} Y${targetY.toFixed(1)} Z${targetZ.toFixed(1)} F12000`);
      
      lastSentX = targetX;
      lastSentY = targetY;
      lastSentZ = targetZ;
      lastSentTime = nowMs;
    }
  }
}

// ─── Image Plotter state (must be declared before initPlotter() is called) ───
const imagePlotter = new ImagePlotter();
const generativePlotter = new GenerativePlotter();
let plotterCamStream = null;
let plottingActive  = false;
let plotCancelled   = false;

// --- Initialize App ---
initEvents();
initPlotter();
initGenerative();
connectWebSocket();
requestAnimationFrame(update);
console.log("Nexus-4 Printer Controller Initialized.");

// ─── Image Plotter UI ────────────────────────────────────────────────────────

function initPlotter() {
  const d = dom;
  imagePlotter.previewCanvas = d.plotterPreview;

  // ── Drag & drop ─────────────────────────────────────────────────────────
  d.plotterDropzone.addEventListener('dragover', (e) => {
    e.preventDefault();
    d.plotterDropzone.style.borderColor = '#00e6ff';
  });
  d.plotterDropzone.addEventListener('dragleave', () => {
    d.plotterDropzone.style.borderColor = '';
  });
  d.plotterDropzone.addEventListener('drop', async (e) => {
    e.preventDefault();
    d.plotterDropzone.style.borderColor = '';
    const file = e.dataTransfer.files[0];
    if (file && file.type.startsWith('image/')) {
      await _plotterLoadFile(file);
    }
  });

  // ── File picker ──────────────────────────────────────────────────────────
  d.plotterBtnFile.addEventListener('click', () => {
    // Reset the input so the same file can be reloaded
    d.plotterFileInput.value = '';
    d.plotterFileInput.click();
  });
  d.plotterFileInput.addEventListener('change', async (e) => {
    const file = e.target.files[0];
    if (file) await _plotterLoadFile(file);
  });

  // ── Camera capture ───────────────────────────────────────────────────────
  d.plotterBtnCamera.addEventListener('click', async () => {
    if (plotterCamStream) {
      // Already open — toggle close
      _stopPlotterCamera();
      return;
    }
    setPlotterStatus('Starting camera…');
    // Pause webcam tracking if active to avoid camera conflict
    if (webcamActive) {
      dom.toggleWebcam.checked = false;
      disableWebcamTracking();
    }
    try {
      plotterCamStream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: 'environment', width: { ideal: 1280 }, height: { ideal: 720 } }
      });
      d.plotterCamVideo.srcObject = plotterCamStream;
      d.plotterCameraArea.style.display = 'block';
      d.plotterBtnCamera.textContent = '❌ Close Camera';
      setPlotterStatus('Camera ready — frame the subject, then snap.');
    } catch (err) {
      setPlotterStatus('Camera error: ' + err.message);
    }
  });

  d.plotterBtnSnap.addEventListener('click', async () => {
    if (!plotterCamStream) return;
    setPlotterStatus('Capturing photo…');
    await imagePlotter.loadFromVideo(d.plotterCamVideo);
    _stopPlotterCamera();
    _plotterImageReady();
  });

  // ── Process ──────────────────────────────────────────────────────────────
  d.plotterBtnProcess.addEventListener('click', () => {
    _plotterRunProcess();
  });

  const _triggerLiveReprocess = () => {
    if (d.plotterMode?.value === 'ml_subject') return; // Do NOT auto-refresh on heavy ML pipeline!
    if (imagePlotter.imageBitmap) _plotterRunProcess();
  };

  // Re-process live when sliders change (except for heavy AI mode)
  d.plotterLines.addEventListener('change',     _triggerLiveReprocess);
  d.plotterThreshold.addEventListener('change', _triggerLiveReprocess);
  if (d.plotterMinStroke) {
    d.plotterMinStroke.addEventListener('change', _triggerLiveReprocess);
  }
  if (d.plotterSimplification) {
    d.plotterSimplification.addEventListener('change', _triggerLiveReprocess);
  }
  if (d.plotterMergeGap) {
    d.plotterMergeGap.addEventListener('change', _triggerLiveReprocess);
  }
  if (d.plotterMode) {
    d.plotterMode.addEventListener('change', () => {
      if (d.plotterMode.value === 'ml_subject') {
        setPlotterStatus('AI Subject Mode selected. Press "Process Image" to run pipeline.');
      } else {
        _triggerLiveReprocess();
      }
    });
  }
  if (d.plotterLineWidth) {
    d.plotterLineWidth.addEventListener('change', _triggerLiveReprocess);
  }
  if (d.plotterStyle) {
    d.plotterStyle.addEventListener('change', _triggerLiveReprocess);
  }
  if (d.plotterShadingDensity) {
    d.plotterShadingDensity.addEventListener('change', _triggerLiveReprocess);
  }

  // ── Plot ─────────────────────────────────────────────────────────────────
  d.plotterBtnPlot.addEventListener('click', async () => {
    if (!imagePlotter.waypoints.length) {
      setPlotterStatus('Process the image first.');
      return;
    }
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      setPlotterStatus('⚠ WebSocket not connected.');
      return;
    }
    plottingActive = true;
    plotCancelled  = false;
    
    // Switch to active controls
    d.plotterBtnPlot.style.display = 'none';
    d.plotterActiveControls.style.display = 'flex';
    d.plotterBtnPause.textContent = '⏸ Pause';
    setPlotterStatus('Plotting…');

    // Show and reset progress display
    let startTime = performance.now();
    d.plotterProgressContainer.style.display = 'flex';
    d.plotterProgressPct.textContent = '0%';
    d.plotterProgressBar.style.width = '0%';
    d.plotterProgressEta.textContent = 'ETA: Calculating...';
    d.plotterProgressCounts.textContent = `0 / ${imagePlotter.waypoints.length} moves`;

    await imagePlotter.streamToPlotter(
      socket,
      /* feedrateXY */ 6000,
      /* feedrateZ  */ 1000,
      (cur, tot, idx) => {
        setPlotterStatus(`Plotting… ${cur}/${tot} moves (${Math.round(cur/tot*100)}%)`);
        
        // Update progress bar
        const pct = Math.round((cur / tot) * 100);
        d.plotterProgressPct.textContent = `${pct}%`;
        d.plotterProgressBar.style.width = `${pct}%`;
        d.plotterProgressCounts.textContent = `${cur} / ${tot} moves`;

        // Calculate running ETA
        if (!imagePlotter._paused) {
          const now = performance.now();
          const elapsedSec = (now - startTime) / 1000;
          if (cur > 5 && elapsedSec > 1) {
            const secPerWp = elapsedSec / cur;
            const remainingSec = Math.round(secPerWp * (tot - cur));
            const m = Math.floor(remainingSec / 60);
            const s = remainingSec % 60;
            d.plotterProgressEta.textContent = `ETA: ${m}:${s < 10 ? '0' : ''}${s}`;
          }
        } else {
          d.plotterProgressEta.textContent = 'ETA: Paused';
        }

        // Render live pen cursor position on the preview canvas
        imagePlotter.renderLivePosition(idx);
      },
      () => plotCancelled
    );

    // Switch back to idle control
    d.plotterBtnPlot.style.display = 'block';
    d.plotterActiveControls.style.display = 'none';
    d.plotterProgressContainer.style.display = 'none';

    plottingActive = false;
    if (!plotCancelled) {
      setPlotterStatus('✅ Plotting complete!');
      // Force draw final frame with cursor at end
      imagePlotter.renderLivePosition(imagePlotter.waypoints.length - 1);
    }
    plotCancelled = false;
  });

  d.plotterBtnPause.addEventListener('click', () => {
    if (imagePlotter._paused) {
      imagePlotter.resume();
      d.plotterBtnPause.textContent = '⏸ Pause';
      setPlotterStatus('Plotting resumed…');
    } else {
      imagePlotter.pause();
      d.plotterBtnPause.textContent = '▶ Resume';
      setPlotterStatus('Plotting paused.');
      d.plotterProgressEta.textContent = 'ETA: Paused';
    }
  });

  d.plotterBtnStop.addEventListener('click', () => {
    plotCancelled = true;
    imagePlotter.stop();
    setPlotterStatus('Plotting stopped.');
  });
}

function _stopPlotterCamera() {
  if (plotterCamStream) {
    plotterCamStream.getTracks().forEach(t => t.stop());
    plotterCamStream = null;
  }
  dom.plotterCameraArea.style.display = 'none';
  dom.plotterBtnCamera.textContent = '📷 Capture';
}

async function _plotterLoadFile(file) {
  setPlotterStatus('Loading image…');
  try {
    await imagePlotter.loadFromFile(file);
    imagePlotter._rawSourceBitmap = imagePlotter.imageBitmap;
    _plotterImageReady();
  } catch (e) {
    setPlotterStatus('Failed to load image: ' + e.message);
  }
}

function _plotterImageReady() {
  dom.plotterBtnProcess.disabled = false;
  dom.plotterBtnPlot.disabled    = true;
  if (dom.plotterMode?.value === 'ml_subject') {
    setPlotterStatus('Image loaded. Press "Process Image" to run AI Subject Line Art pipeline.');
  } else {
    setPlotterStatus('Image loaded. Processing…');
    _plotterRunProcess();
  }
}

function _plotterRunProcess() {
  try {
    const processingMode = dom.plotterMode?.value || 'ml_subject';

    if (processingMode === 'ml_subject') {
      if (!socket || socket.readyState !== WebSocket.OPEN) {
        setPlotterStatus('Error: Server WebSocket not connected. Check backend server.py.');
        return;
      }
      if (!imagePlotter.imageBitmap) {
        setPlotterStatus('No image loaded. Please upload or capture an image first.');
        return;
      }

      setPlotterStatus('🤖 Running PyTorch AI Subject Segmentation on CPU…');

      // Use source image bitmap to create base64 PNG for Python ML pipeline
      const cv = document.createElement('canvas');
      const srcImg = imagePlotter._rawSourceBitmap || imagePlotter.imageBitmap;
      cv.width = srcImg.width;
      cv.height = srcImg.height;
      const ctx = cv.getContext('2d');
      ctx.drawImage(srcImg, 0, 0);
      const b64 = cv.toDataURL('image/png', 0.95);
      const rawB64 = b64.includes(',') ? b64.split(',', 2)[1] : b64;
      const threshold = parseInt(dom.plotterThreshold.value, 10) || 30;

      socket.send(`ml-process:${threshold}:${rawB64}`);
      return;
    }

    setPlotterStatus('Processing…');
    const numLines  = parseInt(dom.plotterLines.value, 10)    || 160;
    const threshold = parseInt(dom.plotterThreshold.value, 10) || 30;
    const minStroke = parseInt(dom.plotterMinStroke?.value, 10) || 8;
    const simplification = parseFloat(dom.plotterSimplification?.value) || 1.5;
    const mergeGapMM = parseFloat(dom.plotterMergeGap?.value) || 4.0;
    const lineWidthMM = parseFloat(dom.plotterLineWidth?.value) || 0.8;
    const drawingStyle = dom.plotterStyle?.value || 'outlines';
    const shadingDensity = parseInt(dom.plotterShadingDensity?.value, 10) || 6;

    // Grab current workspace dimensions from inputs (fall back to handProcessor fields)
    const wW = parseFloat(dom.workspaceWidthInput?.value)  || handProcessor.workspaceWidth  || 200;
    const wH = parseFloat(dom.workspaceHeightInput?.value) || handProcessor.workspaceHeight || 200;

    const workspace = {
      centerX: handProcessor.centerX,
      centerY: handProcessor.centerY,
      workspaceWidth:  wW,
      workspaceHeight: wH
    };

    const count = imagePlotter.process(
      numLines, threshold, minStroke, simplification, mergeGapMM, processingMode, lineWidthMM, drawingStyle, shadingDensity, workspace, penDownZ, penUpZ
    );

    // Show preview
    dom.plotterPreview.style.display = 'block';
    dom.plotterBtnPlot.disabled = false;
    setPlotterStatus(`Ready — ${count} moves, ~${Math.round(count / 5)} path segments. Hit "Start Plotting".`);
  } catch (e) {
    setPlotterStatus('Processing error: ' + e.message);
    console.error(e);
  }
}

function setPlotterStatus(msg) {
  if (dom.plotterStatus) dom.plotterStatus.textContent = msg;
}

// ─── Generative Math Art UI ──────────────────────────────────────────────────

function initGenerative() {
  const d = dom;
  if (!d.generativePreview) return;
  
  generativePlotter.previewCanvas = d.generativePreview;

  // Change parameters labels/values based on selected math pattern
  const setParams = (lbl1, val1, lbl2, val2) => {
    d.lblGenP1.textContent = lbl1;
    d.genP1.value = val1;
    d.lblGenP2.textContent = lbl2;
    d.genP2.value = val2;
  };

  d.genPatternType.addEventListener('change', () => {
    const type = d.genPatternType.value;
    if (type === 'archimedean' || type === 'fermat') {
      setParams('Density (Revs)', 20, 'Scale (Radius mm)', 90);
    } else if (type === 'spirograph') {
      setParams('Inner Gear Rad (mm)', 52, 'Pen Offset (mm)', 38);
    } else if (type === 'rose') {
      setParams('Petal Count (n)', 5, 'Scale (Radius mm)', 90);
    } else if (type === 'lissajous') {
      setParams('X Frequency', 3, 'Y Frequency', 4);
    } else if (type.startsWith('rotate_')) {
      setParams('Copies / Rotations', 24, 'Scale (Radius mm)', 85);
    }
    _generativeRunGenerate();
  });

  // Re-generate preview dynamically when parameters change
  d.genP1.addEventListener('input', _generativeRunGenerate);
  d.genP2.addEventListener('input', _generativeRunGenerate);
  d.genP1.addEventListener('change', _generativeRunGenerate);
  d.genP2.addEventListener('change', _generativeRunGenerate);

  // Generate button
  d.btnGenerativeGenerate.addEventListener('click', () => {
    _generativeRunGenerate();
  });

  // Plot Button
  d.btnGenerativePlot.addEventListener('click', async () => {
    if (!generativePlotter.waypoints.length) {
      d.generativeStatus.textContent = 'Generate art first.';
      return;
    }
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      d.generativeStatus.textContent = '⚠ WebSocket not connected.';
      return;
    }

    plottingActive = true;
    plotCancelled = false;

    // Switch to active controls
    d.btnGenerativePlot.style.display = 'none';
    d.generativeActiveControls.style.display = 'flex';
    d.btnGenerativePause.textContent = '⏸ Pause';
    d.generativeStatus.textContent = 'Plotting mathematical art...';

    // Show and reset progress display
    let startTime = performance.now();
    d.generativeProgressContainer.style.display = 'flex';
    d.generativeProgressPct.textContent = '0%';
    d.generativeProgressBar.style.width = '0%';
    d.generativeProgressEta.textContent = 'ETA: Calculating...';
    d.generativeProgressCounts.textContent = `0 / ${generativePlotter.waypoints.length} moves`;

    const wW = parseFloat(dom.workspaceWidthInput?.value) || 200;
    const wH = parseFloat(dom.workspaceHeightInput?.value) || 200;
    const workspace = {
      centerX: handProcessor.centerX,
      centerY: handProcessor.centerY,
      workspaceWidth: wW,
      workspaceHeight: wH
    };

    await generativePlotter.streamToPlotter(
      socket,
      /* feedrateXY */ 7500, // Faster print speed for clean generator vectors
      /* feedrateZ  */ 1200,
      (cur, tot, idx) => {
        d.generativeStatus.textContent = `Drawing... ${cur}/${tot} points (${Math.round(cur/tot*100)}%)`;
        
        const pct = Math.round((cur / tot) * 100);
        d.generativeProgressPct.textContent = `${pct}%`;
        d.generativeProgressBar.style.width = `${pct}%`;
        d.generativeProgressCounts.textContent = `${cur} / ${tot} moves`;

        if (!generativePlotter._paused) {
          const now = performance.now();
          const elapsedSec = (now - startTime) / 1000;
          if (cur > 5 && elapsedSec > 1) {
            const secPerWp = elapsedSec / cur;
            const remainingSec = Math.round(secPerWp * (tot - cur));
            const m = Math.floor(remainingSec / 60);
            const s = remainingSec % 60;
            d.generativeProgressEta.textContent = `ETA: ${m}:${s < 10 ? '0' : ''}${s}`;
          }
        } else {
          d.generativeProgressEta.textContent = 'ETA: Paused';
        }

        // Live cursor position on the math art canvas
        generativePlotter.renderLivePosition(idx, workspace);
      },
      () => plotCancelled
    );

    // Switch back to idle controls
    d.btnGenerativePlot.style.display = 'block';
    d.generativeActiveControls.style.display = 'none';
    d.generativeProgressContainer.style.display = 'none';

    plottingActive = false;
    if (!plotCancelled) {
      d.generativeStatus.textContent = '✅ Mathematical plotting complete!';
      generativePlotter.renderLivePosition(generativePlotter.waypoints.length - 1, workspace);
    }
    plotCancelled = false;
  });

  d.btnGenerativePause.addEventListener('click', () => {
    if (generativePlotter._paused) {
      generativePlotter.resume();
      d.btnGenerativePause.textContent = '⏸ Pause';
      d.generativeStatus.textContent = 'Plotting resumed...';
    } else {
      generativePlotter.pause();
      d.btnGenerativePause.textContent = '▶ Resume';
      d.generativeStatus.textContent = 'Plotting paused.';
      d.generativeProgressEta.textContent = 'ETA: Paused';
    }
  });

  d.btnGenerativeStop.addEventListener('click', () => {
    plotCancelled = true;
    generativePlotter.stop();
    d.generativeStatus.textContent = 'Plotting stopped.';
  });

  // Pre-generate a default pattern on load
  _generativeRunGenerate();
}

function _generativeRunGenerate() {
  const status = dom.generativeStatus;
  if (!status) return;
  status.textContent = 'Generating...';
  try {
    const type = dom.genPatternType.value;
    const p1 = parseFloat(dom.genP1.value) || 1;
    const p2 = parseFloat(dom.genP2.value) || 1;

    const wW = parseFloat(dom.workspaceWidthInput?.value) || 200;
    const wH = parseFloat(dom.workspaceHeightInput?.value) || 200;
    const workspace = {
      centerX: handProcessor.centerX,
      centerY: handProcessor.centerY,
      workspaceWidth: wW,
      workspaceHeight: wH
    };

    const count = generativePlotter.generate(type, p1, p2, workspace, penDownZ, penUpZ);
    dom.btnGenerativePlot.disabled = false;
    status.textContent = `Generated: ${count} coordinates ready.`;
  } catch (e) {
    status.textContent = 'Error: ' + e.message;
    console.error(e);
  }
}
