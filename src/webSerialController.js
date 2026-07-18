export class Vec3 {
  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }
  add(v) { return new Vec3(this.x + v.x, this.y + v.y, this.z + v.z); }
  sub(v) { return new Vec3(this.x - v.x, this.y - v.y, this.z - v.z); }
  mul(s) { return new Vec3(this.x * s, this.y * s, this.z * s); }
  div(s) { return new Vec3(this.x / s, this.y / s, this.z / s); }
  norm() { return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z); }
  clamp(minV, maxV) {
    return new Vec3(
      Math.max(minV.x, Math.min(maxV.x, this.x)),
      Math.max(minV.y, Math.min(maxV.y, this.y)),
      Math.max(minV.z, Math.min(maxV.z, this.z))
    );
  }
}

export class AlphaBetaEstimator {
  constructor(alpha = 0.92, beta = 0.8, maxExtrap = 0.12, lookaheadBase = 0.04, lookaheadMax = 0.15) {
    this.alpha = alpha;
    this.beta = beta;
    this.maxExtrap = maxExtrap;
    this.lookaheadBase = lookaheadBase;
    this.lookaheadMax = lookaheadMax;

    this.pos = null;
    this.vel = new Vec3();
    this.tLast = null;
    this.tLastReal = null;
    this.tPrevFrame = null;
    this.frameIntervals = [];
  }

  update(meas, t) {
    if (!this.pos) {
      this.pos = meas;
      this.tLast = t;
      this.tLastReal = t;
      this.tPrevFrame = t;
      return;
    }
    
    if (this.tPrevFrame !== null) {
      const interval = t - this.tPrevFrame;
      this.frameIntervals.push(interval);
      if (this.frameIntervals.length > 10) this.frameIntervals.shift();
    }
    this.tPrevFrame = t;

    const dt = Math.max(t - this.tLast, 0.001);
    const pred = this.pos.add(this.vel.mul(dt));
    const residual = meas.sub(pred);
    
    this.pos = pred.add(residual.mul(this.alpha));
    this.vel = this.vel.add(residual.mul(this.beta / dt));
    
    this.tLast = t;
    this.tLastReal = t;
  }

  getAdaptiveLookahead() {
    if (this.frameIntervals.length < 3) return this.lookaheadBase;
    const sum = this.frameIntervals.reduce((a, b) => a + b, 0);
    const mean = sum / this.frameIntervals.length;
    const varSum = this.frameIntervals.reduce((a, b) => a + Math.pow(b - mean, 2), 0);
    const cv = Math.sqrt(varSum / this.frameIntervals.length) / Math.max(mean, 0.001);
    
    const extra = Math.min(this.lookaheadMax - this.lookaheadBase, cv * 0.1);
    return Math.min(this.lookaheadBase + extra, this.lookaheadMax);
  }

  predict(t, lookahead = null) {
    if (!this.pos) return new Vec3(292.0, 387.0, 15.0);

    const look = lookahead !== null ? lookahead : this.getAdaptiveLookahead();
    const dt = Math.min(t - this.tLast + look, this.maxExtrap);

    if (this.tLastReal !== null) {
      const age = t - this.tLastReal;
      const TRACKING_LOSS_S = 0.15;
      if (age > TRACKING_LOSS_S) {
        const decay = Math.max(0.0, 1.0 - (age - TRACKING_LOSS_S) / TRACKING_LOSS_S);
        this.vel = this.vel.mul(decay);
      }
    }

    return this.pos.add(this.vel.mul(dt));
  }
}

export class MotionShaper {
  constructor(maxVel, maxAcc, startPos, softMin, softMax) {
    this.maxVel = maxVel;
    this.maxAcc = maxAcc;
    this.pos = startPos;
    this.vel = new Vec3();
    this.softMin = softMin;
    this.softMax = softMax;
  }

  step(target, dt) {
    let desiredVel = target.sub(this.pos).div(dt);
    const vNorm = desiredVel.norm();
    if (vNorm > this.maxVel) {
      desiredVel = desiredVel.mul(this.maxVel / vNorm);
    }

    let dv = desiredVel.sub(this.vel);
    const dvNorm = dv.norm();
    const maxDv = this.maxAcc * dt;
    if (dvNorm > maxDv) {
      dv = dv.mul(maxDv / dvNorm);
    }

    this.vel = this.vel.add(dv);
    this.pos = this.pos.add(this.vel.mul(dt));
    this.pos = this.pos.clamp(this.softMin, this.softMax);
    return this.pos;
  }
  
  setPosition(pos) {
    this.pos = pos;
  }
}
