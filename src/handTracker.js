// One Euro Filter implementation for noise-free, low-latency coordinate smoothing
class OneEuroFilter {
  constructor(minCutoff = 1.0, beta = 0.007, dcutoff = 1.0) {
    this.minCutoff = minCutoff;
    this.beta = beta;
    this.dcutoff = dcutoff;
    this.xPrev = null;
    this.dxPrev = null;
  }

  filter(x, dt) {
    if (dt <= 0.0001) return this.xPrev !== null ? this.xPrev : x;

    if (this.xPrev === null) {
      this.xPrev = x;
      this.dxPrev = 0;
      return x;
    }

    // 1. Calculate velocity and filter it
    const dx = (x - this.xPrev) / dt;
    const alphaD = this.getAlpha(dt, this.dcutoff);
    const dxFiltered = this.dxPrev + alphaD * (dx - this.dxPrev);
    this.dxPrev = dxFiltered;

    // 2. Compute adaptive cutoff frequency based on velocity
    const cutoff = this.minCutoff + this.beta * Math.abs(dxFiltered);

    // 3. Filter the main signal
    const alpha = this.getAlpha(dt, cutoff);
    const xFiltered = this.xPrev + alpha * (x - this.xPrev);
    this.xPrev = xFiltered;

    return xFiltered;
  }

  getAlpha(dt, cutoff) {
    const tau = 1.0 / (2.0 * Math.PI * cutoff);
    return dt / (dt + tau);
  }

  reset() {
    this.xPrev = null;
    this.dxPrev = null;
  }
}

export class HandTrackingProcessor {
  constructor() {
    // Aggressive filters for X, Y, Z coordinates and wrist tilt pitch
    this.filterX = new OneEuroFilter(0.02, 0.004, 1.0);  // Depth (distance)
    this.filterY = new OneEuroFilter(0.04, 0.006, 1.0);  // Horizontal (yaw)
    this.filterZ = new OneEuroFilter(0.04, 0.006, 1.0);  // Vertical (height)
    this.filterPitch = new OneEuroFilter(0.4, 0.03, 1.0);   // Wrist tilt (highly responsive)

    this.isTracking = false;
    this.lastValidTargets = { x: 120, y: 0, z: 150, pitch: 0 };
  }

  /**
   * Processes raw landmark frames from MediaPipe Hands.
   * Maps coordinates to Robot Cartesian space [X, Y, Z, Pitch]
   */
  processFrame(landmarks, dt) {
    if (!landmarks || landmarks.length < 21) {
      this.isTracking = false;
      return null;
    }

    this.isTracking = true;

    // Landmark 0: Wrist
    // Landmark 9: Middle Finger MCP (center of palm)
    const wrist = landmarks[0];
    const palmCenter = landmarks[9];

    // --- 1. Compute Hand Scale for depth/distance tracking ---
    // Use the 2D palm width (distance between index knuckle 5 and pinky knuckle 17).
    // This is structurally independent of wrist pitch/tilt.
    const indexKnuckle = landmarks[5];
    const pinkyKnuckle = landmarks[17];
    const dScaleX = pinkyKnuckle.x - indexKnuckle.x;
    const dScaleY = pinkyKnuckle.y - indexKnuckle.y;
    const handScale = Math.sqrt(dScaleX * dScaleX + dScaleY * dScaleY);

    // --- 2. Absolute Hand Scale mapping (not relative to rolling windows) ---
    // Expects normalized palm width to span from 0.05 (far from camera) to 0.16 (close to camera).
    // This provides a stable, non-drifting absolute reach coordinate.
    const minS = 0.05;
    const maxS = 0.16;
    const normalizedScale = Math.max(0, Math.min(1, (handScale - minS) / (maxS - minS)));

    // --- 3. Map coordinates to Robot Workspace ---
    // Horizontal: Map Palm X [0, 1] to Robot Y [-250, 250] mm
    const rawTargetY = (0.5 - palmCenter.x) * 550;

    // Vertical: Map Palm Y [0, 1] to Robot Z [-30, 320] mm
    const rawTargetZ = (1.0 - palmCenter.y) * 350 - 30;

    // Depth: Map normalized scale to Robot X [80, 310] mm
    // Small scale (far) -> extended (high Target X)
    // Large scale (close) -> folded (low Target X)
    const rawTargetX = 80 + (1.0 - normalizedScale) * 230;

    // --- 4. Compute Wrist Tilt angle ---
    // Vector from Wrist (0) to Palm Center (9)
    const rawPitchRad = Math.atan2(-(palmCenter.y - wrist.y), palmCenter.x - wrist.x);
    let rawPitchDeg = (rawPitchRad * 180) / Math.PI - 90;
    if (rawPitchDeg < -180) rawPitchDeg += 360;
    rawPitchDeg = Math.max(-80, Math.min(80, rawPitchDeg));

    // --- 5. Apply One Euro Filter to remove noise ---
    const smoothX = this.filterX.filter(rawTargetX, dt);
    const smoothY = this.filterY.filter(rawTargetY, dt);
    const smoothZ = this.filterZ.filter(rawTargetZ, dt);
    const smoothPitch = this.filterPitch.filter(rawPitchDeg, dt);

    // Save final safe values
    this.lastValidTargets = {
      x: Math.max(60, Math.min(320, smoothX)),
      y: Math.max(-300, Math.min(300, smoothY)),
      z: Math.max(-50, Math.min(350, smoothZ)),
      pitch: Math.max(-80, Math.min(80, smoothPitch))
    };

    return this.lastValidTargets;
  }

  reset() {
    this.filterX.reset();
    this.filterY.reset();
    this.filterZ.reset();
    this.filterPitch.reset();
    this.isTracking = false;
  }
}
