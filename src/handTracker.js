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

    // Calculate geometric hand center by averaging key palm landmarks
    const centerIndices = [0, 5, 9, 13, 17];
    let sumX = 0;
    let sumY = 0;
    centerIndices.forEach(idx => {
      sumX += landmarks[idx].x;
      sumY += landmarks[idx].y;
    });
    const handCenterX = sumX / centerIndices.length;
    const handCenterY = sumY / centerIndices.length;

    // --- Map coordinates with an acceleration power curve relative to frame center ---
    // Normalized offset from frame center (-0.5 to +0.5)
    const nx = (1.0 - handCenterX) - 0.5;
    const ny = (1.0 - handCenterY) - 0.5;
    
    // Scale to range [-1.0, 1.0] for exponentiation
    const nxNorm = nx * 2.0;
    const nyNorm = ny * 2.0;
    
    // Apply power exponent (1.4) to create precision deadzone near center and rapid acceleration near edges
    const powerExponent = 1.4;
    const nxCurve = Math.sign(nxNorm) * Math.pow(Math.abs(nxNorm), powerExponent);
    const nyCurve = Math.sign(nyNorm) * Math.pow(Math.abs(nyNorm), powerExponent);
    
    // Scale back to range [0.0, 1.0]
    const targetNx = (nxCurve / 2.0) + 0.5;
    const targetNy = (nyCurve / 2.0) + 0.5;

    // Map to physical printer workspace (200x200mm bounded area)
    const rawTargetX = 192.0 + targetNx * 200.0;
    const rawTargetY = 287.0 + targetNy * 200.0;

    // Dummy Z (actual Z is driven by fistClosed binary check in main.js)
    const rawTargetZ = 15.0;

    // Dummy Pitch
    const rawPitchDeg = 0.0;

    // --- Apply One Euro Filter to remove noise ---
    const smoothX = this.filterX.filter(rawTargetX, dt);
    const smoothY = this.filterY.filter(rawTargetY, dt);
    const smoothZ = this.filterZ.filter(rawTargetZ, dt);
    const smoothPitch = this.filterPitch.filter(rawPitchDeg, dt);

    // Save final safe values centered/clamped in the 200x200mm area
    this.lastValidTargets = {
      x: Math.max(192, Math.min(392, smoothX)),
      y: Math.max(287, Math.min(487, smoothY)),
      z: Math.max(0, Math.min(50, smoothZ)),
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
