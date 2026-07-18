import * as THREE from 'https://unpkg.com/three@0.160.0/build/three.module.js';
import { OrbitControls } from 'https://unpkg.com/three@0.160.0/examples/jsm/controls/OrbitControls.js';
import { 
  degToRad, radToDeg, 
  forwardKinematics, 
  inverseKinematics,
  interpolateJoints
} from './math.js';
import { RobotPhysicsSimulator } from './physics.js';
import { ScrollingChart } from './charts.js';
import { HandTrackingProcessor } from './handTracker.js';
import { WebSerialController } from './webSerialController.js';

// --- Telemetry and Control State ---
let controlMode = 'manual'; // 'manual', 'kinematic', 'planner'
let gravityComp = true;
let traceEnabled = true;

// Joint values: desired targets vs actual
const jointTargets = [0, degToRad(30), degToRad(60), degToRad(-10)];
const physicsSim = new RobotPhysicsSimulator();
physicsSim.reset(jointTargets);

// Kinematic targets
let targetX = 120;
let targetY = 0;
let targetZ = 150;
let targetPitch = 0; // Wrist pitch angle relative to ground

// Raw tracked webcam targets (smoothed continuously in 60Hz loop)
let webcamTargetX = 120;
let webcamTargetY = 0;
let webcamTargetZ = 150;
let webcamTargetPitch = 0;

// Waypoints for Path Planner
let waypoints = [];
let currentWaypointIndex = -1;
let isPlayingPath = false;
let isLoopingPath = false;
let pathProgress = 0;
let pathSpeedScale = 1.0;
let interpolationMode = 'joint';

// --- Web Serial API Client ---
const printerController = new WebSerialController();
let apiConnected = false;

// --- MediaPipe Webcam Tracking ---
let webcamActive = false;
let mediaPipeCamera = null;
let mediaPipeHands = null;
let mediaPipeFaceMesh = null;
let eyeContactRequired = false;
let eyeContactActive = false;
let lastEyeContactTime = 0;
let gazeCalibrated = false;
let calibYaw = 0;
let calibPitch = 0.54; // Default midpoint
let calibLeftGaze = 0;
let calibRightGaze = 0;
let shouldCalibrateGazeNextFrame = false;
let fistClosed = false;
const handProcessor = new HandTrackingProcessor();
let lastTrackedTime = 0;

// --- Three.js Visualizer Globals ---
let scene, camera, renderer, controls;
let robotBase, robotShoulder, robotElbow, robotWrist, robotTip;
let targetMarker, traceLine;
const tracePoints = [];
const maxTracePoints = 250;

// --- Custom Canvas Telemetry Charts ---
const charts = [
  new ScrollingChart(120),
  new ScrollingChart(120),
  new ScrollingChart(120),
  new ScrollingChart(120)
];
const chartCanvasIds = ['chart-j1', 'chart-j2', 'chart-j3', 'chart-j4'];

// Accent Colors matched to CSS
const ACCENT_COLORS = {
  j1: '#00f0ff', // Cyan
  j2: '#39ff14', // Neon Green
  j3: '#ff007f', // Neon Pink
  j4: '#ffaa00'  // Gold
};

// --- DOM References ---
const dom = {
  // Tabs
  tabButtons: document.querySelectorAll('.tab-btn'),
  tabContents: document.querySelectorAll('.tab-content'),
  
  // Accordion
  pidAccordion: document.getElementById('pid-accordion'),
  pidContent: document.getElementById('pid-content'),
  
  // Manual Sliders
  jointSliders: [
    document.getElementById('joint1'),
    document.getElementById('joint2'),
    document.getElementById('joint3'),
    document.getElementById('joint4')
  ],
  jointValTexts: [
    document.getElementById('j1-val'),
    document.getElementById('j2-val'),
    document.getElementById('j3-val'),
    document.getElementById('j4-val')
  ],
  jointActualTexts: [
    document.getElementById('j1-actual'),
    document.getElementById('j2-actual'),
    document.getElementById('j3-actual'),
    document.getElementById('j4-actual')
  ],
  jointErrorTexts: [
    document.getElementById('j1-error'),
    document.getElementById('j2-error'),
    document.getElementById('j3-error'),
    document.getElementById('j4-error')
  ],
  
  // Cartesian Targets
  targetXSlider: document.getElementById('target-x'),
  targetYSlider: document.getElementById('target-y'),
  targetZSlider: document.getElementById('target-z'),
  targetPitchSlider: document.getElementById('target-pitch'),
  txValText: document.getElementById('tx-val'),
  tyValText: document.getElementById('ty-val'),
  tzValText: document.getElementById('tz-val'),
  tpitchValText: document.getElementById('tpitch-val'),
  
  // Telemetry HUD
  telemetryX: document.getElementById('telemetry-x'),
  telemetryY: document.getElementById('telemetry-y'),
  telemetryZ: document.getElementById('telemetry-z'),
  telemetryStatus: document.getElementById('telemetry-status'),
  apiStatus: document.getElementById('api-status'),
  btnConnectPrinter: document.getElementById('btn-connect-printer'),
  ikStatusText: document.getElementById('ik-status-text'),
  ikStatusBox: document.getElementById('ik-status-box'),
  
  hudJ1: document.getElementById('hud-j1'),
  hudJ2: document.getElementById('hud-j2'),
  hudJ3: document.getElementById('hud-j3'),
  hudJ4: document.getElementById('hud-j4'),

  // PID / Configuration
  controlModeSelect: document.getElementById('control-mode-select'),
  pidKp: document.getElementById('pid-kp'),
  pidKi: document.getElementById('pid-ki'),
  pidKd: document.getElementById('pid-kd'),
  kpVal: document.getElementById('kp-val'),
  kiVal: document.getElementById('ki-val'),
  kdVal: document.getElementById('kd-val'),
  
  // Physics Parameters
  physDamping: document.getElementById('phys-damping'),
  physSpeedLimit: document.getElementById('phys-speed-limit'),
  dampingVal: document.getElementById('damping-val'),
  speedLimitVal: document.getElementById('speed-limit-val'),
  toggleGravity: document.getElementById('toggle-gravity'),

  // Waypoint Planner
  btnAddWaypoint: document.getElementById('btn-add-waypoint'),
  btnClearWaypoints: document.getElementById('btn-clear-waypoints'),
  waypointList: document.getElementById('waypoint-list'),
  btnPlayPath: document.getElementById('btn-play-path'),
  btnLoopPath: document.getElementById('btn-loop-path'),
  pathSpeed: document.getElementById('path-speed'),
  speedVal: document.getElementById('speed-val'),
  pathMode: document.getElementById('path-mode'),

  // Viewport Overlay
  btnResetCamera: document.getElementById('btn-reset-camera'),
  btnToggleGrid: document.getElementById('btn-toggle-grid'),
  btnToggleTrace: document.getElementById('btn-toggle-trace'),
  canvasContainer: document.getElementById('canvas-container'),

  // Webcam Tracking
  toggleWebcam: document.getElementById('toggle-webcam'),
  webcamVideo: document.getElementById('webcam-video'),
  webcamOverlay: document.getElementById('webcam-overlay'),
  trackingStatusText: document.getElementById('tracking-status-text'),
  requireEyeContact: document.getElementById('require-eye-contact'),
  eyeContactStatus: document.getElementById('eye-contact-status'),
  btnCalibrateGaze: document.getElementById('btn-calibrate-gaze'),
  gripStatus: document.getElementById('grip-status')
};

// --- Control Settings getters ---
const getControlSettings = () => ({
  kp: parseFloat(dom.pidKp.value),
  ki: parseFloat(dom.pidKi.value),
  kd: parseFloat(dom.pidKd.value),
  damping: parseFloat(dom.physDamping.value),
  speedLimit: degToRad(parseFloat(dom.physSpeedLimit.value)),
  gravityComp: dom.toggleGravity.checked
});

// --- Initialize Web Serial Connection ---
function initSerial() {
  printerController.onStatusChange = (status) => {
    if (status === 'connected') {
      apiConnected = true;
      dom.apiStatus.textContent = 'CONNECTED';
      dom.apiStatus.className = 'value status-badge online';
      dom.btnConnectPrinter.textContent = 'Disconnect Printer';
      dom.btnConnectPrinter.classList.remove('btn-primary');
      dom.btnConnectPrinter.classList.add('btn-danger');
    } else {
      apiConnected = false;
      dom.apiStatus.textContent = 'DISCONNECTED';
      dom.apiStatus.className = 'value status-badge offline';
      dom.btnConnectPrinter.textContent = 'Connect Printer';
      dom.btnConnectPrinter.classList.remove('btn-danger');
      dom.btnConnectPrinter.classList.add('btn-primary');
    }
  };

  dom.btnConnectPrinter.addEventListener('click', async () => {
    if (apiConnected) {
      await printerController.disconnect();
    } else {
      await printerController.connect();
    }
  });
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
        const fk = forwardKinematics(...physicsSim.positions);
        targetX = Math.round(fk.pe.x);
        targetY = Math.round(fk.pe.y);
        targetZ = Math.round(fk.pe.z);
        targetPitch = Math.round(radToDeg(physicsSim.positions[1] + physicsSim.positions[2] + physicsSim.positions[3]));
        
        dom.targetXSlider.value = targetX;
        dom.targetYSlider.value = targetY;
        dom.targetZSlider.value = targetZ;
        dom.targetPitchSlider.value = targetPitch;
        updateCartesianTexts();
      }
    });
  });

  // PID Accordion Toggle
  dom.pidAccordion.addEventListener('click', () => {
    dom.pidContent.classList.toggle('active');
    const arrow = dom.pidAccordion.querySelector('.arrow');
    arrow.style.transform = dom.pidContent.classList.contains('active') ? 'rotate(0deg)' : 'rotate(-90deg)';
  });

  // Control Mode Select (CTC, PID, External)
  dom.controlModeSelect.addEventListener('change', (e) => {
    physicsSim.controlMode = e.target.value;
  });

  // Joint Slider Events
  dom.jointSliders.forEach((slider, idx) => {
    slider.addEventListener('input', (e) => {
      const val = parseInt(e.target.value);
      dom.jointValTexts[idx].textContent = val;
      jointTargets[idx] = degToRad(val);
      
      if (isPlayingPath) pausePathPlayback();
      if (webcamActive) {
        dom.toggleWebcam.checked = false;
        disableWebcamTracking();
      }
    });
  });

  // Cartesian Slider Events
  const onCartesianInput = () => {
    targetX = parseInt(dom.targetXSlider.value);
    targetY = parseInt(dom.targetYSlider.value);
    targetZ = parseInt(dom.targetZSlider.value);
    targetPitch = parseInt(dom.targetPitchSlider.value);
    
    updateCartesianTexts();
    solveIKAndUpdateTargets();
    
    if (isPlayingPath) pausePathPlayback();
    if (webcamActive) {
      dom.toggleWebcam.checked = false;
      disableWebcamTracking();
    }
  };

  dom.targetXSlider.addEventListener('input', onCartesianInput);
  dom.targetYSlider.addEventListener('input', onCartesianInput);
  dom.targetZSlider.addEventListener('input', onCartesianInput);
  dom.targetPitchSlider.addEventListener('input', onCartesianInput);

  // PID / Physics parameters updates
  dom.pidKp.addEventListener('input', (e) => dom.kpVal.textContent = e.target.value);
  dom.pidKi.addEventListener('input', (e) => dom.kiVal.textContent = e.target.value);
  dom.pidKd.addEventListener('input', (e) => dom.kdVal.textContent = e.target.value);
  dom.physDamping.addEventListener('input', (e) => dom.dampingVal.textContent = e.target.value);
  dom.physSpeedLimit.addEventListener('input', (e) => dom.speedLimitVal.textContent = e.target.value);
  
  dom.pathSpeed.addEventListener('input', (e) => {
    pathSpeedScale = parseFloat(e.target.value);
    dom.speedVal.textContent = e.target.value;
  });
  dom.pathMode.addEventListener('change', (e) => {
    interpolationMode = e.target.value;
  });

  dom.toggleGravity.addEventListener('change', (e) => {
    gravityComp = e.target.checked;
  });

  // Waypoints Handlers
  dom.btnAddWaypoint.addEventListener('click', () => {
    const currentTargets = [...jointTargets];
    const fk = forwardKinematics(...currentTargets);
    waypoints.push({
      joints: currentTargets,
      cartesian: { x: fk.pe.x, y: fk.pe.y, z: fk.pe.z, pitch: targetPitch }
    });
    renderWaypointsList();
  });

  dom.btnClearWaypoints.addEventListener('click', () => {
    waypoints = [];
    renderWaypointsList();
    pausePathPlayback();
    tracePoints.length = 0;
    if (traceLine) traceLine.geometry.setFromPoints([]);
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

  // HUD Controls
  dom.btnResetCamera.addEventListener('click', () => {
    camera.position.set(250, 200, 250);
    controls.target.set(0, 80, 0);
    controls.update();
  });

  let gridVisible = true;
  let gridHelper, axesHelper;
  dom.btnToggleGrid.addEventListener('click', () => {
    gridVisible = !gridVisible;
    if (gridHelper) gridHelper.visible = gridVisible;
    if (axesHelper) axesHelper.visible = gridVisible;
  });

  dom.btnToggleTrace.addEventListener('click', () => {
    traceEnabled = !traceEnabled;
    dom.btnToggleTrace.textContent = `Trace: ${traceEnabled ? 'ON' : 'OFF'}`;
    dom.btnToggleTrace.classList.toggle('btn-secondary', !traceEnabled);
    if (!traceEnabled) {
      tracePoints.length = 0;
      if (traceLine) traceLine.geometry.setFromPoints([]);
    }
  });

  // Webcam Tracking Toggle
  dom.toggleWebcam.addEventListener('change', (e) => {
    if (e.target.checked) {
      enableWebcamTracking();
    } else {
      disableWebcamTracking();
    }
  });

  // Require Eye Contact Checkbox Listener
  dom.requireEyeContact.addEventListener('change', (e) => {
    eyeContactRequired = e.target.checked;
    if (!eyeContactRequired) {
      eyeContactActive = false;
      dom.eyeContactStatus.textContent = 'NO CONTACT';
      dom.eyeContactStatus.className = 'status-badge offline';
    }
  });

  // Calibrate Gaze Button Listener
  dom.btnCalibrateGaze.addEventListener('click', () => {
    if (!webcamActive) {
      alert('Start webcam tracking first before calibrating gaze!');
      return;
    }
    shouldCalibrateGazeNextFrame = true;
    dom.btnCalibrateGaze.textContent = 'Calibrating...';
    dom.btnCalibrateGaze.style.background = 'rgba(255, 170, 0, 0.2)';
    dom.btnCalibrateGaze.style.borderColor = '#ffaa00';
    dom.btnCalibrateGaze.style.color = '#ffaa00';
  });
}

function updateCartesianTexts() {
  dom.txValText.textContent = Math.round(targetX);
  dom.tyValText.textContent = Math.round(targetY);
  dom.tzValText.textContent = Math.round(targetZ);
  dom.tpitchValText.textContent = Math.round(targetPitch);
}

function solveIKAndUpdateTargets() {
  const pitchRad = degToRad(targetPitch);
  const solution = inverseKinematics(targetX, targetY, targetZ, pitchRad, true);
  
  if (solution) {
    dom.ikStatusText.textContent = 'VALID SOLUTION';
    dom.ikStatusBox.classList.remove('negative');
    
    for (let i = 0; i < 4; i++) {
      jointTargets[i] = solution[i];
      const degVal = Math.round(radToDeg(solution[i]));
      dom.jointSliders[i].value = degVal;
      dom.jointValTexts[i].textContent = degVal;
    }
  } else {
    dom.ikStatusText.textContent = 'OUT OF WORKSPACE';
    dom.ikStatusBox.classList.add('negative');
  }
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
    
    const x = Math.round(wp.cartesian.x);
    const y = Math.round(wp.cartesian.y);
    const z = Math.round(wp.cartesian.z);
    
    li.innerHTML = `
      <div>
        <span class="wp-num">WP #${idx + 1}</span>
        <span class="wp-coords mono">(${x}, ${y}, ${z}) p:${Math.round(wp.cartesian.pitch)}°</span>
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
      
      for (let i = 0; i < 4; i++) {
        jointTargets[i] = wp.joints[i];
        const degVal = Math.round(radToDeg(wp.joints[i]));
        dom.jointSliders[i].value = degVal;
        dom.jointValTexts[i].textContent = degVal;
      }
      
      const targetPos = forwardKinematics(...wp.joints).pe;
      targetMarker.position.set(targetPos.x, targetPos.z, targetPos.y);
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
// --- MediaPipe Hand & Face Tracking Setup ---
function enableWebcamTracking() {
  if (webcamActive) return;

  dom.trackingStatusText.textContent = 'INITIALIZING...';
  dom.trackingStatusText.className = 'tracking-status lost';

  // 1. Initialize MediaPipe Hands object
  try {
    if (!mediaPipeHands) {
      mediaPipeHands = new window.Hands({
        locateFile: (file) => `https://cdn.jsdelivr.net/npm/@mediapipe/hands/${file}`
      });

      mediaPipeHands.setOptions({
        maxNumHands: 1,
        modelComplexity: 1,
        minDetectionConfidence: 0.6,
        minTrackingConfidence: 0.6
      });

      mediaPipeHands.onResults(onHandResults);
    }

    // 1.5 Initialize MediaPipe Face Mesh object for Gaze Tracking
    if (!mediaPipeFaceMesh) {
      mediaPipeFaceMesh = new window.FaceMesh({
        locateFile: (file) => `https://cdn.jsdelivr.net/npm/@mediapipe/face_mesh/${file}`
      });

      mediaPipeFaceMesh.setOptions({
        maxNumFaces: 1,
        refineLandmarks: true, // Enables precise iris tracking for true pupil eye contact detection
        minDetectionConfidence: 0.5,
        minTrackingConfidence: 0.5
      });

      mediaPipeFaceMesh.onResults(onFaceResults);
    }

    // 2. Open Webcam stream using MediaPipe Camera utility
    navigator.mediaDevices.getUserMedia({ video: { width: 640, height: 480 } })
      .then((stream) => {
        dom.webcamVideo.srcObject = stream;
        webcamActive = true;
        
        mediaPipeCamera = new window.Camera(dom.webcamVideo, {
          onFrame: async () => {
            if (webcamActive) {
              // Clear overlay canvas once at the start of the frame to prevent draw races
              const canvas = dom.webcamOverlay;
              const ctx = canvas.getContext('2d');
              if (ctx) ctx.clearRect(0, 0, canvas.width, canvas.height);

              await mediaPipeHands.send({ image: dom.webcamVideo });
              if (mediaPipeFaceMesh) {
                await mediaPipeFaceMesh.send({ image: dom.webcamVideo });
              }
            }
          },
          width: 640,
          height: 480
        });
        
        mediaPipeCamera.start();
        
        // Force switch control mode tab to Kinematic Cartesian control
        const kinTabButton = document.querySelector('.tab-btn[data-tab="kinematic"]');
        if (kinTabButton) kinTabButton.click();
      })
      .catch((err) => {
        console.error('Error starting camera stream:', err);
        alert('Webcam permission denied or camera not found.');
        dom.toggleWebcam.checked = false;
        disableWebcamTracking();
      });
  } catch (err) {
    console.error('MediaPipe initialization error:', err);
    alert('Failed to load MediaPipe Hands/FaceMesh libraries.');
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

  // Clear overlay canvas
  const canvas = dom.webcamOverlay;
  const ctx = canvas.getContext('2d');
  if (ctx) ctx.clearRect(0, 0, canvas.width, canvas.height);

  handProcessor.reset();
  
  dom.trackingStatusText.textContent = 'TRACKING OFF';
  dom.trackingStatusText.className = 'tracking-status lost';

  // Reset Gaze Status
  eyeContactActive = false;
  dom.eyeContactStatus.textContent = 'NO CONTACT';
  dom.eyeContactStatus.className = 'status-badge offline';

  // Reset Grip Status
  fistClosed = false;
  dom.gripStatus.textContent = 'OPEN';
  dom.gripStatus.className = 'status-badge offline';
  dom.gripStatus.style.background = 'rgba(139, 148, 158, 0.15)';
  dom.gripStatus.style.color = '#8b949e';
  dom.gripStatus.style.borderColor = 'rgba(139, 148, 158, 0.3)';
}

function onFaceResults(results) {
  if (!webcamActive) return;

  const canvas = dom.webcamOverlay;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;

  const width = canvas.width;
  const height = canvas.height;

  if (results.multiFaceLandmarks && results.multiFaceLandmarks.length > 0) {
    const landmarks = results.multiFaceLandmarks[0];

    // --- 1. Compute Head Pose (Yaw and Pitch) ---
    const noseTip = landmarks[1];
    const leftEyeInner = landmarks[133];
    const rightEyeInner = landmarks[362];
    const leftEyeOuter = landmarks[33];
    const rightEyeOuter = landmarks[263];
    const forehead = landmarks[10];
    const chin = landmarks[152];

    // Head Yaw Asymmetry (Nose relative to horizontal eye center)
    const eyeCenter = (leftEyeInner.x + rightEyeInner.x) / 2;
    const yawOffset = noseTip.x - eyeCenter;
    const eyeDistance = Math.sqrt(
      (rightEyeInner.x - leftEyeInner.x) ** 2 + 
      (rightEyeInner.y - leftEyeInner.y) ** 2
    );
    const normalizedYaw = yawOffset / (eyeDistance + 1e-6);

    // Head Pitch (Nose vertical ratio in face height)
    const faceHeight = chin.y - forehead.y;
    const noseRelativeY = (noseTip.y - forehead.y) / (faceHeight + 1e-6);

    // --- 2. Compute Pupil Gaze (horizontal centering in eye sockets) ---
    let leftGazeOffset = 0;
    let rightGazeOffset = 0;
    const irisesPresent = landmarks.length > 473;

    if (irisesPresent) {
      const leftIris = landmarks[468];
      const rightIris = landmarks[473];

      const leftEyeWidth = Math.abs(leftEyeOuter.x - leftEyeInner.x);
      const leftEyeCenter = (leftEyeOuter.x + leftEyeInner.x) / 2;
      leftGazeOffset = (leftIris.x - leftEyeCenter) / (leftEyeWidth + 1e-6);

      const rightEyeWidth = Math.abs(rightEyeOuter.x - rightEyeInner.x);
      const rightEyeCenter = (rightEyeOuter.x + rightEyeInner.x) / 2;
      rightGazeOffset = (rightIris.x - rightEyeCenter) / (rightEyeWidth + 1e-6);
    }

    // --- 3. Gaze Calibration Handler ---
    if (shouldCalibrateGazeNextFrame) {
      calibYaw = normalizedYaw;
      calibPitch = noseRelativeY;
      calibLeftGaze = leftGazeOffset;
      calibRightGaze = rightGazeOffset;
      gazeCalibrated = true;
      shouldCalibrateGazeNextFrame = false;

      // Update UI button style to show calibration complete
      dom.btnCalibrateGaze.textContent = 'Calibrated';
      dom.btnCalibrateGaze.style.background = 'rgba(57, 255, 20, 0.15)';
      dom.btnCalibrateGaze.style.borderColor = '#39ff14';
      dom.btnCalibrateGaze.style.color = '#39ff14';
    }

    // --- 4. Evaluate Eye Contact with Adaptive Boundaries ---
    let headValid = false;
    let gazeValid = false;

    if (gazeCalibrated) {
      // Much more generous tolerances relative to calibrated center to allow comfortable movement
      headValid = Math.abs(normalizedYaw - calibYaw) < 0.45 && Math.abs(noseRelativeY - calibPitch) < 0.20;
      gazeValid = !irisesPresent || (Math.abs(leftGazeOffset - calibLeftGaze) < 0.35 && Math.abs(rightGazeOffset - calibRightGaze) < 0.35);
    } else {
      // Relaxed baseline tolerances before calibration
      headValid = Math.abs(normalizedYaw) < 0.45 && Math.abs(noseRelativeY - 0.54) < 0.20;
      gazeValid = !irisesPresent || (Math.abs(leftGazeOffset) < 0.45 && Math.abs(rightGazeOffset) < 0.45);
    }

    const gazeLock = headValid && gazeValid;

    if (gazeLock) {
      eyeContactActive = true;
      lastEyeContactTime = performance.now(); // Reset memory timer
    } else {
      // Stop instantly when user looks away
      eyeContactActive = false;
    }

    // --- 3. Update Gaze status badge ---
    if (eyeContactRequired) {
      if (eyeContactActive) {
        dom.eyeContactStatus.textContent = 'GAZE LOCKED';
        dom.eyeContactStatus.className = 'status-badge'; // green
      } else {
        dom.eyeContactStatus.textContent = 'LOOK AT SCREEN';
        dom.eyeContactStatus.className = 'status-badge offline'; // red
      }
    } else {
      dom.eyeContactStatus.textContent = eyeContactActive ? 'GAZE LOCKED' : 'NO CONTACT';
      dom.eyeContactStatus.className = eyeContactActive ? 'status-badge' : 'status-badge offline';
    }

    // --- 4. Draw Reticles on Overlay Canvas ---
    ctx.strokeStyle = eyeContactActive ? '#39ff14' : '#ef4444';
    ctx.lineWidth = 2;
    ctx.shadowBlur = 6;
    ctx.shadowColor = eyeContactActive ? '#39ff14' : '#ef4444';

    // Draw eye boundaries (targeting reticles)
    [leftEyeInner, rightEyeInner].forEach((eye) => {
      ctx.beginPath();
      ctx.arc(eye.x * width, eye.y * height, 15, 0, 2 * Math.PI);
      ctx.stroke();
    });

    // Draw pupil tracking iris dots
    if (irisesPresent) {
      ctx.fillStyle = eyeContactActive ? '#39ff14' : '#ef4444';
      [landmarks[468], landmarks[473]].forEach((iris) => {
        ctx.beginPath();
        ctx.arc(iris.x * width, iris.y * height, 4, 0, 2 * Math.PI);
        ctx.fill();
      });
    }

    // HUD Text indicator
    ctx.fillStyle = eyeContactActive ? '#39ff14' : '#ef4444';
    ctx.font = 'bold 10px JetBrains Mono';
    ctx.fillText(
      eyeContactActive ? 'SYS_GAZE: LOCKED' : 'SYS_GAZE: SEARCHING', 
      10, 
      canvas.height - 15
    );
    
    // Draw debugging telemetry values on overlay canvas (futuristic diagnostic overlay)
    ctx.fillStyle = 'rgba(10, 12, 16, 0.75)';
    ctx.fillRect(10, 10, 240, 60);
    ctx.lineWidth = 1;
    ctx.strokeStyle = eyeContactActive ? 'rgba(57, 255, 20, 0.3)' : 'rgba(239, 68, 68, 0.3)';
    ctx.strokeRect(10, 10, 240, 60);
    
    ctx.fillStyle = '#ffffff';
    ctx.font = '8px JetBrains Mono';
    
    const yawColor = headValid ? '#39ff14' : '#ef4444';
    const pitchColor = headValid ? '#39ff14' : '#ef4444';
    const lGazeColor = gazeValid ? '#39ff14' : '#ef4444';
    const rGazeColor = gazeValid ? '#39ff14' : '#ef4444';

    const refYawText = gazeCalibrated ? `${calibYaw >= 0 ? '+' : ''}${calibYaw.toFixed(2)}` : '0.00';
    const refPitchText = gazeCalibrated ? `${calibPitch.toFixed(2)}` : '0.54';
    const refLGazeText = gazeCalibrated ? `${calibLeftGaze >= 0 ? '+' : ''}${calibLeftGaze.toFixed(2)}` : '0.00';
    const refRGazeText = gazeCalibrated ? `${calibRightGaze >= 0 ? '+' : ''}${calibRightGaze.toFixed(2)}` : '0.00';

    const yawTol = gazeCalibrated ? '0.45' : '0.45';
    const pitchTol = gazeCalibrated ? '0.20' : '0.20';
    const gazeTol = gazeCalibrated ? '0.35' : '0.45';

    ctx.fillStyle = '#8b949e'; ctx.fillText('HEAD YAW   :', 18, 22);
    ctx.fillStyle = yawColor;    ctx.fillText(`${normalizedYaw >= 0 ? '+' : ''}${normalizedYaw.toFixed(2)} (ref: ${refYawText}, tol: ${yawTol})`, 95, 22);
    
    ctx.fillStyle = '#8b949e'; ctx.fillText('HEAD PITCH :', 18, 33);
    ctx.fillStyle = pitchColor;  ctx.fillText(`${noseRelativeY.toFixed(2)} (ref: ${refPitchText}, tol: ${pitchTol})`, 95, 33);
    
    ctx.fillStyle = '#8b949e'; ctx.fillText('L_PUPIL_DX :', 18, 44);
    ctx.fillStyle = lGazeColor;  ctx.fillText(`${leftGazeOffset >= 0 ? '+' : ''}${leftGazeOffset.toFixed(2)} (ref: ${refLGazeText}, tol: ${gazeTol})`, 95, 44);
    
    ctx.fillStyle = '#8b949e'; ctx.fillText('R_PUPIL_DX :', 18, 55);
    ctx.fillStyle = rGazeColor;  ctx.fillText(`${rightGazeOffset >= 0 ? '+' : ''}${rightGazeOffset.toFixed(2)} (ref: ${refRGazeText}, tol: ${gazeTol})`, 95, 55);

    // If calibrated, write a tiny green [CALIB] status badge in top-right of overlay box
    if (gazeCalibrated) {
      ctx.fillStyle = '#39ff14';
      ctx.font = 'bold 7px JetBrains Mono';
      ctx.fillText('[CALIB]', 205, 18);
    }

    ctx.shadowBlur = 0;
  } else {
    // Face lost (e.g. hand blocking face or user out of frame).
    // Apply 1.8-second cooldown (hysteresis) to prevent robot freezing.
    const elapsed = performance.now() - lastEyeContactTime;
    
    if (webcamActive && elapsed < 1800) {
      eyeContactActive = true; // Keep control active during cooldown
      
      // Update UI to notify user that blockage memory is active
      dom.eyeContactStatus.textContent = 'GAZE COOLDOWN';
      dom.eyeContactStatus.className = 'status-badge online'; // cyan/blue-ish

      // Draw a yellow warning reticle indicating tracking memory is active
      ctx.strokeStyle = '#ffaa00';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.arc(width / 2, height / 2, 40, 0, 2 * Math.PI);
      ctx.stroke();
      ctx.setLineDash([]);
      
      ctx.fillStyle = '#ffaa00';
      ctx.font = 'bold 10px JetBrains Mono';
      ctx.fillText(`SYS_GAZE: MEMORY_HOLD (${((1800 - elapsed) / 1000).toFixed(1)}s)`, 10, canvas.height - 15);
    } else {
      eyeContactActive = false;
      dom.eyeContactStatus.textContent = 'NO FACE';
      dom.eyeContactStatus.className = 'status-badge offline'; // red
    }
  }
}

function onHandResults(results) {
  if (!webcamActive) return;

  const canvas = dom.webcamOverlay;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;

  // Sync canvas size with video size
  const video = dom.webcamVideo;
  if (canvas.width !== video.videoWidth || canvas.height !== video.videoHeight) {
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;
  }

  // Clear is now handled once per frame in the Camera onFrame callback!

  const timestamp = performance.now();
  // Reset timing tracking if we just regained hand tracking to prevent delta spikes
  if (!handProcessor.isTracking || lastTrackedTime === 0) {
    lastTrackedTime = timestamp;
  }
  const dt = (timestamp - lastTrackedTime) / 1000;
  lastTrackedTime = timestamp;

  if (results.multiHandLandmarks && results.multiHandLandmarks.length > 0) {
    // Hand detected!
    dom.trackingStatusText.textContent = 'ACTIVE';
    dom.trackingStatusText.className = 'tracking-status active';

    const landmarks = results.multiHandLandmarks[0];

    // Detect if fist is closed
    fistClosed = checkFistClosed(landmarks);

    // Update Grip Status UI Indicator
    if (fistClosed) {
      dom.gripStatus.textContent = 'CLOSED';
      dom.gripStatus.className = 'status-badge';
      dom.gripStatus.style.background = 'rgba(255, 170, 0, 0.15)'; // gold
      dom.gripStatus.style.color = '#ffaa00';
      dom.gripStatus.style.borderColor = 'rgba(255, 170, 0, 0.3)';
    } else {
      dom.gripStatus.textContent = 'OPEN';
      dom.gripStatus.className = 'status-badge offline';
      dom.gripStatus.style.background = 'rgba(139, 148, 158, 0.15)';
      dom.gripStatus.style.color = '#8b949e';
      dom.gripStatus.style.borderColor = 'rgba(139, 148, 158, 0.3)';
    }

    // Draw hand skeleton skeleton overlay
    drawHandSkeleton(ctx, landmarks);

    // Process coordinates with One Euro Filter and dynamic self-calibration
    const targets = handProcessor.processFrame(landmarks, dt);

    if (targets) {
      webcamTargetX = targets.x;
      webcamTargetY = targets.y;
      webcamTargetZ = targets.z;
      webcamTargetPitch = targets.pitch;
    }
  } else {
    // Tracking lost
    dom.trackingStatusText.textContent = 'LOST';
    dom.trackingStatusText.className = 'tracking-status lost';
    handProcessor.isTracking = false;

    // Reset Grip status
    fistClosed = false;
    dom.gripStatus.textContent = 'OPEN';
    dom.gripStatus.className = 'status-badge offline';
    dom.gripStatus.style.background = 'rgba(139, 148, 158, 0.15)';
    dom.gripStatus.style.color = '#8b949e';
    dom.gripStatus.style.borderColor = 'rgba(139, 148, 158, 0.3)';
  }
}

function checkFistClosed(landmarks) {
  // Normalize finger curl distances by hand palm size (wrist landmark 0 to middle finger knuckle landmark 9)
  const dx = landmarks[9].x - landmarks[0].x;
  const dy = landmarks[9].y - landmarks[0].y;
  const dz = landmarks[9].z - landmarks[0].z;
  const handSize = Math.sqrt(dx*dx + dy*dy + dz*dz) + 1e-6;

  // Calculate distances from fingertips to knuckle bases
  const indexDist = Math.sqrt((landmarks[8].x - landmarks[5].x)**2 + (landmarks[8].y - landmarks[5].y)**2 + (landmarks[8].z - landmarks[5].z)**2) / handSize;
  const middleDist = Math.sqrt((landmarks[12].x - landmarks[9].x)**2 + (landmarks[12].y - landmarks[9].y)**2 + (landmarks[12].z - landmarks[9].z)**2) / handSize;
  const ringDist = Math.sqrt((landmarks[16].x - landmarks[13].x)**2 + (landmarks[16].y - landmarks[13].y)**2 + (landmarks[16].z - landmarks[13].z)**2) / handSize;
  const pinkyDist = Math.sqrt((landmarks[20].x - landmarks[17].x)**2 + (landmarks[20].y - landmarks[17].y)**2 + (landmarks[20].z - landmarks[17].z)**2) / handSize;

  // Fist is closed if all four finger tips are curled tightly close to knuckles
  return indexDist < 0.50 && middleDist < 0.50 && ringDist < 0.50 && pinkyDist < 0.50;
}

function drawHandSkeleton(ctx, landmarks) {
  const width = ctx.canvas.width;
  const height = ctx.canvas.height;

  // Draw connectors (cyan for open, gold for closed grasp)
  ctx.strokeStyle = fistClosed ? '#ffaa00' : '#00f0ff';
  ctx.lineWidth = 3;
  ctx.shadowBlur = 4;
  ctx.shadowColor = fistClosed ? '#ffaa00' : '#00f0ff';

  const connections = [
    // Thumb
    [0, 1], [1, 2], [2, 3], [3, 4],
    // Index
    [0, 5], [5, 6], [6, 7], [7, 8],
    // Middle
    [0, 9], [9, 10], [10, 11], [11, 12],
    // Ring
    [0, 13], [13, 14], [14, 15], [15, 16],
    // Pinky
    [0, 17], [17, 18], [18, 19], [19, 20],
    // Palm connections
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

  // Draw joints (neon pink for open, gold for closed grasp)
  ctx.fillStyle = fistClosed ? '#ffaa00' : '#ff007f';
  ctx.shadowColor = fistClosed ? '#ffaa00' : '#ff007f';
  landmarks.forEach((pt) => {
    ctx.beginPath();
    ctx.arc(pt.x * width, pt.y * height, 5, 0, 2 * Math.PI);
    ctx.fill();
  });

  // Reset shadow
  ctx.shadowBlur = 0;
}

// --- Initialize Three.js Scene ---
function initThree() {
  const container = dom.canvasContainer;
  const width = container.clientWidth;
  const height = container.clientHeight;

  scene = new THREE.Scene();
  scene.background = new THREE.Color('#0a0c10');

  camera = new THREE.PerspectiveCamera(45, width / height, 1, 1000);
  camera.position.set(250, 200, 250);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(width, height);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.target.set(0, 80, 0);

  const ambientLight = new THREE.AmbientLight('#2b2f3d', 0.8);
  scene.add(ambientLight);

  const dirLight = new THREE.DirectionalLight('#ffffff', 1.0);
  dirLight.position.set(200, 400, 200);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.width = 1024;
  dirLight.shadow.mapSize.height = 1024;
  dirLight.shadow.camera.near = 0.5;
  dirLight.shadow.camera.far = 800;
  const d = 250;
  dirLight.shadow.camera.left = -d;
  dirLight.shadow.camera.right = d;
  dirLight.shadow.camera.top = d;
  dirLight.shadow.camera.bottom = -d;
  scene.add(dirLight);

  const pointLight = new THREE.PointLight('#00f0ff', 1.2, 300);
  pointLight.position.set(0, 150, 0);
  scene.add(pointLight);

  // Keep references to toggle grid
  const gridHelper = new THREE.GridHelper(500, 50, '#1c2230', '#131824');
  gridHelper.position.y = -0.5;
  scene.add(gridHelper);

  const axesHelper = new THREE.AxesHelper(100);
  axesHelper.position.y = 0.1;
  scene.add(axesHelper);

  // Materials
  const metalMaterial = new THREE.MeshStandardMaterial({
    color: '#303440',
    metalness: 0.85,
    roughness: 0.25
  });

  const neonJointRing = (colorHex) => {
    return new THREE.MeshStandardMaterial({
      color: colorHex,
      emissive: colorHex,
      emissiveIntensity: 0.4,
      metalness: 0.5,
      roughness: 0.2
    });
  };

  // Build robot meshes matching math dimensions (converting meters to UI millimeters/drawing units)
  robotBase = new THREE.Group();
  scene.add(robotBase);

  // base stand
  const baseStand = new THREE.Mesh(new THREE.CylinderGeometry(35, 40, 80, 32), metalMaterial);
  baseStand.position.y = 40;
  baseStand.castShadow = true;
  baseStand.receiveShadow = true;
  robotBase.add(baseStand);

  const baseRing = new THREE.Mesh(new THREE.CylinderGeometry(36, 36, 6, 32), neonJointRing(ACCENT_COLORS.j1));
  baseRing.position.y = 77;
  robotBase.add(baseRing);

  robotShoulder = new THREE.Group();
  robotShoulder.position.y = 80;
  robotBase.add(robotShoulder);

  const shoulderCap = new THREE.Mesh(new THREE.SphereGeometry(22, 32, 16), metalMaterial);
  shoulderCap.castShadow = true;
  robotShoulder.add(shoulderCap);

  const shoulderAxisMesh = new THREE.Mesh(new THREE.CylinderGeometry(8, 8, 48, 16), neonJointRing(ACCENT_COLORS.j2));
  shoulderAxisMesh.rotation.z = Math.PI / 2;
  robotShoulder.add(shoulderAxisMesh);

  // Link 2 Upper Arm (140 units)
  const upperArmLink = new THREE.Mesh(new THREE.BoxGeometry(16, 140, 24), metalMaterial);
  upperArmLink.position.y = 70;
  upperArmLink.castShadow = true;
  upperArmLink.receiveShadow = true;
  robotShoulder.add(upperArmLink);

  robotElbow = new THREE.Group();
  robotElbow.position.y = 140;
  robotShoulder.add(robotElbow);

  const elbowCap = new THREE.Mesh(new THREE.SphereGeometry(18, 32, 16), metalMaterial);
  elbowCap.castShadow = true;
  robotElbow.add(elbowCap);

  const elbowAxisMesh = new THREE.Mesh(new THREE.CylinderGeometry(6, 6, 38, 16), neonJointRing(ACCENT_COLORS.j3));
  elbowAxisMesh.rotation.z = Math.PI / 2;
  robotElbow.add(elbowAxisMesh);

  // Link 3 Forearm (120 units)
  const forearmLink = new THREE.Mesh(new THREE.BoxGeometry(12, 120, 18), metalMaterial);
  forearmLink.position.y = 60;
  forearmLink.castShadow = true;
  forearmLink.receiveShadow = true;
  robotElbow.add(forearmLink);

  robotWrist = new THREE.Group();
  robotWrist.position.y = 120;
  robotElbow.add(robotWrist);

  const wristCap = new THREE.Mesh(new THREE.SphereGeometry(12, 16, 16), metalMaterial);
  wristCap.castShadow = true;
  robotWrist.add(wristCap);

  const wristAxisMesh = new THREE.Mesh(new THREE.CylinderGeometry(4, 4, 26, 16), neonJointRing(ACCENT_COLORS.j4));
  wristAxisMesh.rotation.z = Math.PI / 2;
  robotWrist.add(wristAxisMesh);

  // Link 4 Tool Hand (60 units)
  const wristLink = new THREE.Mesh(new THREE.BoxGeometry(8, 60, 12), metalMaterial);
  wristLink.position.y = 30;
  wristLink.castShadow = true;
  robotWrist.add(wristLink);

  const toolTip = new THREE.Mesh(
    new THREE.SphereGeometry(6, 16, 16),
    new THREE.MeshBasicMaterial({ color: '#ffaa00' })
  );
  toolTip.position.y = 60;
  robotWrist.add(toolTip);
  robotTip = toolTip;

  // Target Location Indicator
  const targetGeom = new THREE.SphereGeometry(8, 16, 16);
  const targetMat = new THREE.MeshBasicMaterial({
    color: '#00f0ff',
    wireframe: true,
    transparent: true,
    opacity: 0.6
  });
  targetMarker = new THREE.Mesh(targetGeom, targetMat);
  scene.add(targetMarker);

  // Trace line
  const traceMaterial = new THREE.LineBasicMaterial({
    color: '#ff007f',
    linewidth: 2
  });
  const traceGeom = new THREE.BufferGeometry();
  traceLine = new THREE.Line(traceGeom, traceMaterial);
  scene.add(traceLine);

  window.addEventListener('resize', () => {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  });
}

// --- Main Loop ---
let lastTime = 0;

function update(time) {
  requestAnimationFrame(update);

  if (lastTime === 0) {
    lastTime = time;
    return;
  }

  let dt = (time - lastTime) / 1000;
  if (dt < 0.001) return; // Skip frame updates that occur too quickly, accumulating time step
  if (dt > 0.1) dt = 0.1; // Protect simulation against focus loss drops
  lastTime = time;

  // 1. Process Path Planner sequencing
  if (isPlayingPath && waypoints.length >= 2) {
    const speed = pathSpeedScale;
    pathProgress += dt * 0.25 * speed;
    
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
        if (interpolationMode === 'joint') {
          const interpolated = interpolateJoints(startWp.joints, endWp.joints, pathProgress);
          for (let i = 0; i < 4; i++) {
            jointTargets[i] = interpolated[i];
            const degVal = Math.round(radToDeg(jointTargets[i]));
            dom.jointSliders[i].value = degVal;
            dom.jointValTexts[i].textContent = degVal;
          }
        } else {
          const sx = startWp.cartesian.x;
          const sy = startWp.cartesian.y;
          const sz = startWp.cartesian.z;
          const spitch = startWp.cartesian.pitch;
          
          const ex = endWp.cartesian.x;
          const ey = endWp.cartesian.y;
          const ez = endWp.cartesian.z;
          const epitch = endWp.cartesian.pitch;

          targetX = sx + (ex - sx) * pathProgress;
          targetY = sy + (ey - sy) * pathProgress;
          targetZ = sz + (ez - sz) * pathProgress;
          targetPitch = spitch + (epitch - spitch) * pathProgress;

          dom.targetXSlider.value = targetX;
          dom.targetYSlider.value = targetY;
          dom.targetZSlider.value = targetZ;
          dom.targetPitchSlider.value = targetPitch;
          updateCartesianTexts();
          solveIKAndUpdateTargets();
        }
      }
    }
  }

  // 1.5 Smooth webcam targets at 60Hz to eliminate target discretization step-jitter
  const allowTrackingUpdate = !eyeContactRequired || (eyeContactRequired && eyeContactActive);
  if (webcamActive && handProcessor.isTracking && allowTrackingUpdate) {
    const k_pos = 12.0;    // Bandwidth for coordinates X, Y, Z (smooth and natural)
    const k_pitch = 28.0;  // Bandwidth for wrist pitch (tuned for extreme responsiveness/low latency)
    
    targetX += (webcamTargetX - targetX) * (1 - Math.exp(-k_pos * dt));
    targetY += (webcamTargetY - targetY) * (1 - Math.exp(-k_pos * dt));
    targetZ += (webcamTargetZ - targetZ) * (1 - Math.exp(-k_pos * dt));
    targetPitch += (webcamTargetPitch - targetPitch) * (1 - Math.exp(-k_pitch * dt));
    
    solveIKAndUpdateTargets();
    updateCartesianTexts();
    
    // Sync DOM slider positions
    dom.targetXSlider.value = Math.round(targetX);
    dom.targetYSlider.value = Math.round(targetY);
    dom.targetZSlider.value = Math.round(targetZ);
    dom.targetPitchSlider.value = Math.round(targetPitch);
  }

  // 2. Advance coupled physical dynamics simulation step
  const ctrlSettings = getControlSettings();
  physicsSim.step(jointTargets, ctrlSettings, dt);

  // 3. Sync Three.js mesh transformations
  robotBase.rotation.y = physicsSim.positions[0];
  robotShoulder.rotation.z = physicsSim.positions[1] - Math.PI / 2; // Offset vertical alignment
  robotElbow.rotation.z = physicsSim.positions[2];
  robotWrist.rotation.z = physicsSim.positions[3];

  // 4. Update HUD and telemetry outputs
  const actualFK = forwardKinematics(...physicsSim.positions);
  dom.telemetryX.textContent = actualFK.pe.x.toFixed(1);
  dom.telemetryY.textContent = actualFK.pe.y.toFixed(1);
  dom.telemetryZ.textContent = actualFK.pe.z.toFixed(1);

  dom.hudJ1.textContent = (radToDeg(physicsSim.positions[0])).toFixed(1) + '°';
  dom.hudJ2.textContent = (radToDeg(physicsSim.positions[1])).toFixed(1) + '°';
  dom.hudJ3.textContent = (radToDeg(physicsSim.positions[2])).toFixed(1) + '°';
  dom.hudJ4.textContent = (radToDeg(physicsSim.positions[3])).toFixed(1) + '°';

  let maxError = 0;
  for (let i = 0; i < 4; i++) {
    const actDeg = radToDeg(physicsSim.positions[i]);
    const tarDeg = radToDeg(jointTargets[i]);
    const errDeg = actDeg - tarDeg;
    
    dom.jointActualTexts[i].textContent = Math.round(actDeg);
    dom.jointErrorTexts[i].textContent = errDeg.toFixed(1);
    maxError = Math.max(maxError, Math.abs(errDeg));
  }

  // Set Status Telemetry Badge
  const telemetryStatus = dom.telemetryStatus;
  if (physicsSim.controlMode === 'external_torque') {
    telemetryStatus.textContent = 'EXTERNAL CONTROL';
    telemetryStatus.style.background = 'rgba(255, 0, 127, 0.15)';
    telemetryStatus.style.color = '#ff007f';
    telemetryStatus.style.borderColor = 'rgba(255, 0, 127, 0.3)';
  } else if (physicsSim.controlMode === 'local_stepper' && !webcamActive && !isPlayingPath) {
    telemetryStatus.textContent = 'STEPPER OPEN-LOOP';
    telemetryStatus.style.background = 'rgba(0, 240, 255, 0.15)';
    telemetryStatus.style.color = '#00f0ff';
    telemetryStatus.style.borderColor = 'rgba(0, 240, 255, 0.3)';
  } else if (webcamActive && handProcessor.isTracking) {
    if (eyeContactRequired && !eyeContactActive) {
      telemetryStatus.textContent = 'PAUSED - NO EYE CONTACT';
      telemetryStatus.style.background = 'rgba(255, 170, 0, 0.15)';
      telemetryStatus.style.color = '#ffaa00';
      telemetryStatus.style.borderColor = 'rgba(255, 170, 0, 0.3)';
    } else {
      telemetryStatus.textContent = 'WEBCAM TRACKING';
      telemetryStatus.style.background = 'rgba(57, 255, 20, 0.15)';
      telemetryStatus.style.color = '#39ff14';
      telemetryStatus.style.borderColor = 'rgba(57, 255, 20, 0.3)';
    }
  } else if (isPlayingPath) {
    telemetryStatus.textContent = 'EXECUTING PATH';
    telemetryStatus.style.background = 'rgba(0, 240, 255, 0.15)';
    telemetryStatus.style.color = '#00f0ff';
    telemetryStatus.style.borderColor = 'rgba(0, 240, 255, 0.3)';
  } else if (maxError > 2.0) {
    telemetryStatus.textContent = 'ADJUSTING';
    telemetryStatus.style.background = 'rgba(255, 170, 0, 0.15)';
    telemetryStatus.style.color = '#ffaa00';
    telemetryStatus.style.borderColor = 'rgba(255, 170, 0, 0.3)';
  } else {
    telemetryStatus.textContent = 'STANDBY';
    telemetryStatus.style.background = 'rgba(16, 185, 129, 0.15)';
    telemetryStatus.style.color = '#10b981';
    telemetryStatus.style.borderColor = 'rgba(16, 185, 129, 0.3)';
  }

  // Update target marker in 3D scene
  const targetFK = forwardKinematics(...jointTargets);
  targetMarker.position.set(targetFK.pe.x, targetFK.pe.z, targetFK.pe.y);

  // Update path trace line
  if (traceEnabled) {
    const tipWorldPos = new THREE.Vector3();
    robotTip.getWorldPosition(tipWorldPos);
    
    tracePoints.push(tipWorldPos.clone());
    if (tracePoints.length > maxTracePoints) {
      tracePoints.shift();
    }
    traceLine.geometry.setFromPoints(tracePoints);
  }

  // Render scrolling canvas charts
  for (let i = 0; i < 4; i++) {
    const tarDeg = radToDeg(jointTargets[i]);
    const actDeg = radToDeg(physicsSim.positions[i]);
    charts[i].addSample(tarDeg, actDeg);
    
    const canvas = document.getElementById(chartCanvasIds[i]);
    charts[i].draw(canvas, '#8b949e', Object.values(ACCENT_COLORS)[i]);
    
    const errDeg = actDeg - tarDeg;
    document.getElementById(`chart-err-j${i+1}`).textContent = (errDeg >= 0 ? '+' : '') + errDeg.toFixed(1) + '°';
  }

  // 5. Update Web Serial Controller with new Target directly (Zero latency)
  if (apiConnected) {
    const pen = fistClosed ? 1.0 : 0.0;
    
    // Instead of streaming JSON over a WebSocket, we inject the Cartesian targets 
    // directly into the AlphaBeta filter running on the WebSerialController.
    printerController.updateTarget(targetX, targetY, targetZ, pen);
  }

  controls.update();
  renderer.render(scene, camera);
}

// --- Initialize App ---
initEvents();
initThree();
initSerial();
requestAnimationFrame(update);
console.log("Nexus-4 Advanced Simulation Controller Initialized Successfully.");
