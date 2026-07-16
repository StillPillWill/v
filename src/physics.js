// Robot dimensions in meters for physics calculations
export const l1 = 0.08;   // Base height
export const l2 = 0.14;   // Upper arm
export const l3 = 0.12;   // Forearm
export const l4 = 0.06;   // Hand / Tool

// Link Masses (kg)
export const m1 = 5.0;    // Base
export const m2 = 3.0;    // Upper arm
export const m3 = 2.0;    // Forearm
export const m4 = 1.0;    // Hand / Tool
const g = 9.81;           // Gravity acceleration

export const jointParams = [
  { name: "Base", minLimit: -Math.PI, maxLimit: Math.PI, maxTorque: 40.0 },
  { name: "Shoulder", minLimit: -Math.PI / 2, maxLimit: (2 * Math.PI) / 3, maxTorque: 50.0 },
  { name: "Elbow", minLimit: -(5 * Math.PI) / 6, maxLimit: (5 * Math.PI) / 6, maxTorque: 30.0 },
  { name: "Wrist", minLimit: -(2 * Math.PI) / 3, maxLimit: (2 * Math.PI) / 3, maxTorque: 15.0 }
];

/**
 * Solves the linear system A * x = B using Gaussian Elimination (for 4x4 matrix inversion).
 */
function solveLinearSystem4x4(A, B) {
  const n = 4;
  const M = Array.from({ length: n }, () => new Float64Array(n + 1));
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      M[i][j] = A[i][j];
    }
    M[i][n] = B[i];
  }

  for (let i = 0; i < n; i++) {
    let maxRow = i;
    for (let k = i + 1; k < n; k++) {
      if (Math.abs(M[k][i]) > Math.abs(M[maxRow][i])) {
        maxRow = k;
      }
    }

    const temp = M[i];
    M[i] = M[maxRow];
    M[maxRow] = temp;

    if (Math.abs(M[i][i]) < 1e-12) {
      return null;
    }

    for (let k = i + 1; k < n; k++) {
      const factor = M[k][i] / M[i][i];
      for (let j = i; j <= n; j++) {
        M[k][j] -= factor * M[i][j];
      }
    }
  }

  const x = new Float64Array(n);
  for (let i = n - 1; i >= 0; i--) {
    let sum = M[i][n];
    for (let j = i + 1; j < n; j++) {
      sum -= M[i][j] * x[j];
    }
    x[i] = sum / M[i][i];
  }
  return x;
}

export class RobotPhysicsSimulator {
  constructor() {
    // Current actual states
    this.positions = [0.0, (30 * Math.PI) / 180, (60 * Math.PI) / 180, (-10 * Math.PI) / 180];
    this.velocities = [0.0, 0.0, 0.0, 0.0];
    
    // Model Reference Trajectory States (Smooth 2nd-order target generator)
    this.qRef = [...this.positions];
    this.dqRef = [0.0, 0.0, 0.0, 0.0];
    this.ddqRef = [0.0, 0.0, 0.0, 0.0];
    
    this.appliedTorques = [0.0, 0.0, 0.0, 0.0];
    
    // Joint PID variables
    this.pidIntegrals = [0.0, 0.0, 0.0, 0.0];

    // Control override mode
    this.controlMode = 'local_ctc'; // 'local_ctc', 'local_pid', 'external_torque'
  }

  reset(positions = [0.0, 0.5, 1.0, -0.2]) {
    this.positions = [...positions];
    this.velocities = [0.0, 0.0, 0.0, 0.0];
    this.qRef = [...positions];
    this.dqRef.fill(0.0);
    this.ddqRef.fill(0.0);
    this.appliedTorques.fill(0.0);
    this.pidIntegrals.fill(0.0);
  }

  /**
   * Computes the 4x4 coupled Inertia Matrix M(q)
   */
  getInertiaMatrix(q) {
    const [t1, t2, t3, t4] = q;

    const rG2 = 0.5 * l2 * Math.cos(t2);
    const rG3 = l2 * Math.cos(t2) + 0.5 * l3 * Math.cos(t2 + t3);
    const rG4 = l2 * Math.cos(t2) + l3 * Math.cos(t2 + t3) + 0.5 * l4 * Math.cos(t2 + t3 + t4);

    const I1 = 0.15;
    const M11 = I1 + m2 * rG2 * rG2 + m3 * rG3 * rG3 + m4 * rG4 * rG4;

    const M44 = 0.015;
    const M33 = 0.065 + m4 * l3 * l4 * Math.cos(t4) + m4 * 0.25 * l4 * l4;
    const M22 = 0.18 + (m3 + m4) * l2 * l3 * Math.cos(t3) + m4 * l2 * l4 * Math.cos(t3 + t4);

    const M23 = 0.045 * Math.cos(t3) + 0.012 * Math.cos(t3 + t4);
    const M24 = 0.012 * Math.cos(t3 + t4);
    const M34 = 0.008 * Math.cos(t4);

    return [
      [M11, 0, 0, 0],
      [0, M22, M23, M24],
      [0, M23, M33, M34],
      [0, M24, M34, M44]
    ];
  }

  /**
   * Computes the Coriolis and Centrifugal torque vector C(q, dq) * dq
   */
  getCoriolisVector(q, dq) {
    const [t1, t2, t3, t4] = q;
    const [dq1, dq2, dq3, dq4] = dq;

    const rG2 = 0.5 * l2 * Math.cos(t2);
    const rG3 = l2 * Math.cos(t2) + 0.5 * l3 * Math.cos(t2 + t3);
    const rG4 = l2 * Math.cos(t2) + l3 * Math.cos(t2 + t3) + 0.5 * l4 * Math.cos(t2 + t3 + t4);

    const drG2_dt2 = -0.5 * l2 * Math.sin(t2);
    const drG3_dt2 = -l2 * Math.sin(t2) - 0.5 * l3 * Math.sin(t2 + t3);
    const drG3_dt3 = -0.5 * l3 * Math.sin(t2 + t3);
    const drG4_dt2 = -l2 * Math.sin(t2) - l3 * Math.sin(t2 + t3) - 0.5 * l4 * Math.sin(t2 + t3 + t4);
    const drG4_dt3 = -l3 * Math.sin(t2 + t3) - 0.5 * l4 * Math.sin(t2 + t3 + t4);
    const drG4_dt4 = -0.5 * l4 * Math.sin(t2 + t3 + t4);

    const dM11_dt2 = 2 * (m2 * rG2 * drG2_dt2 + m3 * rG3 * drG3_dt2 + m4 * rG4 * drG4_dt2);
    const dM11_dt3 = 2 * (m3 * rG3 * drG3_dt3 + m4 * rG4 * drG4_dt3);
    const dM11_dt4 = 2 * (m4 * rG4 * drG4_dt4);

    const tc1_centrif = dM11_dt2 * dq1 * dq2 + dM11_dt3 * dq1 * dq3 + dM11_dt4 * dq1 * dq4;
    const tc2_centrif = -0.5 * dM11_dt2 * dq1 * dq1;
    const tc3_centrif = -0.5 * dM11_dt3 * dq1 * dq1;
    const tc4_centrif = -0.5 * dM11_dt4 * dq1 * dq1;

    const tc2_coriolis = -0.015 * Math.sin(t3) * dq3 * (2 * dq2 + dq3);
    const tc3_coriolis = 0.015 * Math.sin(t3) * dq2 * dq2 - 0.005 * Math.sin(t4) * dq4 * (2 * dq2 + 2 * dq3 + dq4);
    const tc4_coriolis = 0.005 * Math.sin(t4) * (dq2 + dq3) * (dq2 + dq3);

    return [
      tc1_centrif,
      tc2_centrif + tc2_coriolis,
      tc3_centrif + tc3_coriolis,
      tc4_centrif + tc4_coriolis
    ];
  }

  /**
   * Computes the gravity torque loading vector G(q)
   */
  getGravityVector(q) {
    const [t1, t2, t3, t4] = q;

    const G4 = m4 * g * (0.5 * l4) * Math.cos(t2 + t3 + t4);
    const G3 = (m3 * g * (0.5 * l3) + m4 * g * l3) * Math.cos(t2 + t3) + G4;
    const G2 = (m2 * g * (0.5 * l2) + (m3 + m4) * g * l2) * Math.cos(t2) + G3;
    const G1 = 0;

    return [G1, G2, G3, G4];
  }

  /**
   * Computed Torque Control (CTC) law with reference inputs:
   * tau = M(q) * (ddq_ref + Kp*e + Kd*de) + C(q,dq)*dq + G(q)
   */
  computeComputedTorque(kp, kd, gravityCompEnabled) {
    const q = this.positions;
    const dq = this.velocities;

    const M = this.getInertiaMatrix(q);
    const C_dq = this.getCoriolisVector(q, dq);
    const G = this.getGravityVector(q);

    const torques = new Float64Array(4);
    const u = new Float64Array(4);

    for (let i = 0; i < 4; i++) {
      let error = this.qRef[i] - q[i];
      if (i === 0) {
        error = Math.atan2(Math.sin(error), Math.cos(error));
      }
      const dError = this.dqRef[i] - dq[i];
      u[i] = this.ddqRef[i] + kp * error + kd * dError;
    }

    for (let i = 0; i < 4; i++) {
      let mu = 0;
      for (let j = 0; j < 4; j++) {
        mu += M[i][j] * u[j];
      }
      
      const tg = gravityCompEnabled ? G[i] : 0;
      torques[i] = mu + C_dq[i] + tg;
    }

    return torques;
  }

  /**
   * Standard decoupled PID controller
   */
  computeDecoupledPID(targets, kp, ki, kd, gravityCompEnabled, dt) {
    const q = this.positions;
    const dq = this.velocities;
    const G = this.getGravityVector(q);
    const torques = new Float64Array(4);

    for (let i = 0; i < 4; i++) {
      let error = targets[i] - q[i];
      if (i === 0) {
        error = Math.atan2(Math.sin(error), Math.cos(error));
      }

      this.pidIntegrals[i] += error * dt;
      this.pidIntegrals[i] = Math.max(-5.0, Math.min(5.0, this.pidIntegrals[i]));

      const pTerm = kp * error;
      const iTerm = ki * this.pidIntegrals[i];
      const dTerm = kd * (-dq[i]);

      const tg = gravityCompEnabled ? G[i] : 0;
      torques[i] = pTerm + iTerm + dTerm + tg;
    }
    return torques;
  }

  /**
   * Sub-stepping architecture wraps the integration step to ensure absolute stability
   * @param {Array<number>} targets - Raw desired targets
   * @param {object} controlSettings - Control gains and limit settings
   * @param {number} dt - Frame time delta
   */
  step(targets, controlSettings, dt) {
    if (dt <= 0) return;

    // Split frame delta into 1ms sub-steps to guarantee numerical integration stability
    const maxSubStep = 0.001; 
    const numSteps = Math.ceil(dt / maxSubStep);
    const dtSub = dt / numSteps;

    for (let step = 0; step < numSteps; step++) {
      this.singleStep(targets, controlSettings, dtSub);
    }
  }

  /**
   * Runs a single integration step of size dt (guaranteed to be tiny, <= 1ms)
   */
  singleStep(targets, controlSettings, dt) {
    // 0. Stepper Motor Emulation Mode (Open-loop trajectory profiling)
    if (this.controlMode === 'local_stepper') {
      const speedLimit = controlSettings.speedLimit;
      const maxAccel = 10.0; // rad/s^2 (corresponds to AccelStepper limits)
      
      for (let i = 0; i < 4; i++) {
        let err = targets[i] - this.positions[i];
        if (i === 0) {
          err = Math.atan2(Math.sin(err), Math.cos(err));
        }
        
        // Target velocity to eliminate error in 1 step
        const targetVel = err / (dt + 1e-6);
        
        // Calculate acceleration needed
        const reqAccel = (targetVel - this.velocities[i]) / (dt + 1e-6);
        const clampedAccel = Math.max(-maxAccel, Math.min(maxAccel, reqAccel));
        
        // Integrate velocity & position
        this.velocities[i] += clampedAccel * dt;
        this.velocities[i] = Math.max(-speedLimit, Math.min(speedLimit, this.velocities[i]));
        this.positions[i] += this.velocities[i] * dt;
        
        // Clamp to joint limits
        if (this.positions[i] <= jointParams[i].minLimit) {
          this.positions[i] = jointParams[i].minLimit;
          this.velocities[i] = 0;
        } else if (this.positions[i] >= jointParams[i].maxLimit) {
          this.positions[i] = jointParams[i].maxLimit;
          this.velocities[i] = 0;
        }
        
        // Sync reference model
        this.qRef[i] = this.positions[i];
        this.dqRef[i] = this.velocities[i];
        this.ddqRef[i] = clampedAccel;
      }
      return;
    }

    // 1. Model Reference Trajectory Generator (extremely stable at small dt)
    const zeta = 1.0; // Critical damping
    
    for (let i = 0; i < 4; i++) {
      let errRef = targets[i] - this.qRef[i];
      if (i === 0) {
        errRef = Math.atan2(Math.sin(errRef), Math.cos(errRef));
      }
      
      // Joint-dependent trajectory bandwidth: wrist (i=3) has high bandwidth for snappiness
      const omega = (i === 3) ? 30.0 : 12.0;
      
      // Calculate smooth target acceleration
      this.ddqRef[i] = omega * omega * errRef - 2.0 * zeta * omega * this.dqRef[i];
      this.dqRef[i] += this.ddqRef[i] * dt;
      this.qRef[i] += this.dqRef[i] * dt;
    }

    // 2. Select Control Scheme
    let controlTorques;
    
    if (this.controlMode === 'external_torque') {
      controlTorques = [...this.appliedTorques];
    } else if (this.controlMode === 'local_pid') {
      controlTorques = this.computeDecoupledPID(
        targets, 
        controlSettings.kp, 
        controlSettings.ki, 
        controlSettings.kd, 
        controlSettings.gravityComp, 
        dt
      );
    } else {
      // CTC with critical damping auto-enforcer
      const kp = controlSettings.kp;
      const kd_crit = 2.0 * Math.sqrt(kp);
      
      controlTorques = this.computeComputedTorque(
        kp, 
        kd_crit, 
        controlSettings.gravityComp
      );
    }

    // 3. Saturate joint torques (motor physical limits)
    for (let i = 0; i < 4; i++) {
      const limit = jointParams[i].maxTorque;
      this.appliedTorques[i] = Math.max(-limit, Math.min(limit, controlTorques[i]));
    }

    // 4. Integrate physical dynamics
    const q = this.positions;
    const dq = this.velocities;
    
    const M = this.getInertiaMatrix(q);
    const C_dq = this.getCoriolisVector(q, dq);
    const G = this.getGravityVector(q);

    const residual = new Float64Array(4);
    const damping = controlSettings.damping;

    for (let i = 0; i < 4; i++) {
      // Damping scaled by diagonal entry M[i][i] is stable for sub-stepping
      residual[i] = this.appliedTorques[i] - C_dq[i] - G[i] - damping * M[i][i] * dq[i];
    }

    const ddq = solveLinearSystem4x4(M, residual);
    
    if (ddq) {
      const speedLimit = controlSettings.speedLimit;
      
      for (let i = 0; i < 4; i++) {
        this.velocities[i] += ddq[i] * dt;
        this.velocities[i] = Math.max(-speedLimit, Math.min(speedLimit, this.velocities[i]));
        
        this.positions[i] += this.velocities[i] * dt;

        if (this.positions[i] <= jointParams[i].minLimit) {
          this.positions[i] = jointParams[i].minLimit;
          this.velocities[i] = 0;
        } else if (this.positions[i] >= jointParams[i].maxLimit) {
          this.positions[i] = jointParams[i].maxLimit;
          this.velocities[i] = 0;
        }
      }
    }
  }
}
