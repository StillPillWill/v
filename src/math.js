// Robot Arm Link Lengths (in mm)
export const L1 = 80;   // Base height (ground to shoulder)
export const L2 = 140;  // Upper arm length (shoulder to elbow)
export const L3 = 120;  // Forearm length (elbow to wrist)
export const L4 = 60;   // Wrist length (wrist to end effector tip)

// Convert degrees to radians
export function degToRad(deg) {
  return (deg * Math.PI) / 180;
}

// Convert radians to degrees
export function radToDeg(rad) {
  return (rad * 180) / Math.PI;
}

/**
 * Computes Forward Kinematics (FK) for the 4-axis arm.
 * Returns the 3D positions of all joints and the end effector.
 * @param {number} t1 - Base yaw (rad)
 * @param {number} t2 - Shoulder pitch (rad)
 * @param {number} t3 - Elbow pitch (rad)
 * @param {number} t4 - Wrist pitch (rad)
 */
export function forwardKinematics(t1, t2, t3, t4) {
  // Base Joint (Joint 0)
  const p0 = { x: 0, y: 0, z: 0 };
  
  // Shoulder Joint (Joint 1)
  const p1 = { x: 0, y: 0, z: L1 };
  
  // 2D planar coordinates (radius, height) relative to shoulder
  // Elbow Joint (Joint 2)
  const r2 = L2 * Math.cos(t2);
  const z2 = L1 + L2 * Math.sin(t2);
  const p2 = {
    x: r2 * Math.cos(t1),
    y: r2 * Math.sin(t1),
    z: z2
  };
  
  // Wrist Joint (Joint 3)
  const r3 = r2 + L3 * Math.cos(t2 + t3);
  const z3 = z2 + L3 * Math.sin(t2 + t3);
  const p3 = {
    x: r3 * Math.cos(t1),
    y: r3 * Math.sin(t1),
    z: z3
  };
  
  // End Effector Tip
  const re = r3 + L4 * Math.cos(t2 + t3 + t4);
  const ze = z3 + L4 * Math.sin(t2 + t3 + t4);
  const pe = {
    x: re * Math.cos(t1),
    y: re * Math.sin(t1),
    z: ze
  };
  
  return { p0, p1, p2, p3, pe };
}

/**
 * Computes Inverse Kinematics (IK) for the 4-axis arm.
 * Returns joint angles [t1, t2, t3, t4] in radians or null if unreachable.
 * @param {number} x - Target X (mm)
 * @param {number} y - Target Y (mm)
 * @param {number} z - Target Z (mm)
 * @param {number} phi - Wrist orientation angle relative to horizontal plane (rad)
 * @param {boolean} elbowUp - Solve for elbow-up configuration (default: true)
 */
export function inverseKinematics(x, y, z, phi, elbowUp = true) {
  // 1. Base yaw angle (t1)
  const t1 = Math.atan2(y, x);
  
  // 2. Project target point into 2D arm plane (r, z)
  const r = Math.sqrt(x * x + y * y);
  
  // 3. Subtract wrist link (L4) vector to get target wrist position (rw, zw)
  const rw = r - L4 * Math.cos(phi);
  const zw = z - L4 * Math.sin(phi);
  
  // 4. Solve 2-link planar arm (L2, L3) from shoulder (0, L1) to wrist (rw, zw)
  const r_prime = rw;
  const z_prime = zw - L1;
  
  // Square of distance from shoulder to wrist
  const D2 = r_prime * r_prime + z_prime * z_prime;
  const D = Math.sqrt(D2);
  
  // Check if position is out of workspace
  if (D > L2 + L3 || D < Math.abs(L2 - L3) || D === 0) {
    return null; // Position is unreachable
  }
  
  // Law of cosines for elbow angle (t3)
  const cos_t3 = (D2 - L2 * L2 - L3 * L3) / (2 * L2 * L3);
  
  // Check for numerical rounding issues
  const clamped_cos_t3 = Math.max(-1, Math.min(1, cos_t3));
  
  // Elbow angle (t3)
  const t3 = elbowUp 
    ? -Math.acos(clamped_cos_t3) // Elbow up (negative angle bends upwards)
    : Math.acos(clamped_cos_t3);  // Elbow down (positive angle bends downwards)
    
  // Shoulder angle (t2)
  const t2 = Math.atan2(z_prime, r_prime) - Math.atan2(L3 * Math.sin(t3), L2 + L3 * Math.cos(t3));
  
  // 5. Wrist angle (t4)
  const t4 = phi - t2 - t3;
  
  // 6. Check joint limit bounds (roughly matched to UI sliders)
  // Base: -180 to 180 deg
  // Shoulder: -90 to 120 deg
  // Elbow: -150 to 150 deg
  // Wrist: -120 to 120 deg
  const t2_deg = radToDeg(t2);
  const t3_deg = radToDeg(t3);
  const t4_deg = radToDeg(t4);
  
  if (t2_deg < -91 || t2_deg > 121 || 
      t3_deg < -151 || t3_deg > 151 || 
      t4_deg < -121 || t4_deg > 121) {
    return null; // Solution exists mathematically but violates joint range limits
  }
  
  return [t1, t2, t3, t4];
}

/**
 * Interpolates between two sets of joint angles
 * @param {Array<number>} start - [t1, t2, t3, t4] in radians
 * @param {Array<number>} end - [t1, t2, t3, t4] in radians
 * @param {number} fraction - Interpolation value between 0 and 1
 */
export function interpolateJoints(start, end, fraction) {
  return start.map((startVal, idx) => {
    // Handle angle wrapping for base joint (t1) specifically if needed, 
    // but simple linear interpolation works for standard waypoint movements
    return startVal + (end[idx] - startVal) * fraction;
  });
}
