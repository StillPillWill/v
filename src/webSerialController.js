class Vec3 {
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

class AlphaBetaEstimator {
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
    if (!this.pos) return new Vec3(585.0 / 2, 775.0 / 2, 15.0);

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

class MotionShaper {
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

export class WebSerialController {
  constructor() {
    this.port = null;
    this.writer = null;
    this.reader = null;
    this.keepReading = true;

    this.maxInFlight = 3;
    this.inFlight = 0;
    this.isHoming = false;
    this.printerHomed = false;

    this.bedX = 585.0;
    this.bedY = 775.0;

    this.estimator = new AlphaBetaEstimator();
    this.shaper = new MotionShaper(
      200.0, // MAX_VEL_MM_S
      1500.0, // MAX_ACC_MM_S2
      new Vec3(this.bedX / 2, this.bedY / 2, 15.0),
      new Vec3(0, 0, 0),
      new Vec3(this.bedX, this.bedY, 50.0)
    );

    this.lastSentX = null;
    this.lastSentY = null;
    this.targetPen = 1.0;
    this.lastSentPen = 1.0;

    this.controlHz = 250;
    this.controlDt = 1.0 / this.controlHz;
    this.controlLoopTimer = null;
    this.onStatusChange = null;
  }

  async connect() {
    if (!navigator.serial) {
      alert("Web Serial API is not supported in this browser. Please use Chrome or Edge.");
      return false;
    }

    try {
      this.port = await navigator.serial.requestPort();
      await this.port.open({ baudRate: 115200 });

      const textEncoder = new TextEncoderStream();
      textEncoder.readable.pipeTo(this.port.writable);
      this.writer = textEncoder.writable.getWriter();

      this.startReading();

      // Clear errors & absolute mode
      await this.sendInstant("M999");
      await new Promise(r => setTimeout(r, 300));
      await this.sendInstant("G90");

      if (!this.printerHomed) {
        console.log("[SERIAL] Sending G28 home...");
        this.isHoming = true;
        await this.sendInstant("G28");
        this.printerHomed = true;
        
        setTimeout(() => {
          this.estimator.pos = new Vec3(this.bedX / 2, this.bedY / 2, 15.0);
          this.estimator.vel = new Vec3(0, 0, 0);
          this.shaper.setPosition(new Vec3(this.bedX / 2, this.bedY / 2, 15.0));
          this.shaper.vel = new Vec3(0, 0, 0);
          this.isHoming = false;
          console.log("[SERIAL] Homing presumed complete.");
        }, 10000); // 10 seconds approximation for homing
      }

      this.startControlLoop();
      if (this.onStatusChange) this.onStatusChange("connected");
      return true;
    } catch (e) {
      console.error("[SERIAL ERROR]", e);
      alert("Failed to connect to printer: " + e.message);
      if (this.onStatusChange) this.onStatusChange("disconnected");
      return false;
    }
  }

  async disconnect() {
    if (this.controlLoopTimer) {
      clearInterval(this.controlLoopTimer);
      this.controlLoopTimer = null;
    }
    this.keepReading = false;
    if (this.reader) {
      await this.reader.cancel();
    }
    if (this.writer) {
      await this.sendInstant("M410");
      this.writer.releaseLock();
    }
    if (this.port) {
      await this.port.close();
      this.port = null;
    }
    if (this.onStatusChange) this.onStatusChange("disconnected");
  }

  async startReading() {
    const textDecoder = new TextDecoderStream();
    this.port.readable.pipeTo(textDecoder.writable);
    this.reader = textDecoder.readable
      .pipeThrough(new TransformStream(new LineBreakTransformer()))
      .getReader();

    try {
      while (this.keepReading) {
        const { value, done } = await this.reader.read();
        if (done) break;
        if (value && value.toLowerCase().startsWith("ok")) {
          this.inFlight = Math.max(0, this.inFlight - 1);
        }
      }
    } catch (e) {
      console.error("[SERIAL READER ERROR]", e);
    } finally {
      if (this.reader) this.reader.releaseLock();
    }
  }

  async sendInstant(cmd) {
    if (this.writer) {
      await this.writer.write(cmd + "\n");
    }
  }

  async send(cmd) {
    if (this.inFlight >= this.maxInFlight) return false;
    this.inFlight++;
    try {
      await this.writer.write(cmd + "\n");
      return true;
    } catch (e) {
      this.inFlight = Math.max(0, this.inFlight - 1);
      return false;
    }
  }

  updateTarget(x, y, z, pen) {
    const now = performance.now() / 1000.0;
    this.targetPen = pen;
    this.estimator.update(new Vec3(x, y, z), now);
  }

  startControlLoop() {
    this.controlLoopTimer = setInterval(() => {
      if (!this.port || this.isHoming) return;

      const now = performance.now() / 1000.0;

      // Handle Pen State
      if (this.targetPen !== this.lastSentPen) {
        this.sendInstant("M410");
        this.inFlight = 0; // Clear in-flight
        setTimeout(() => {
          const cmd = this.targetPen === 1.0 ? "G91\nG1 Z4 F99999\nG90" : "G1 Z0 F99999";
          const lines = cmd.split("\n");
          lines.forEach(line => {
            if (line.trim()) this.sendInstant(line.trim());
          });
        }, 50);
        this.lastSentPen = this.targetPen;
        return;
      }

      const estimated = this.estimator.predict(now);
      const safePos = this.shaper.step(estimated, this.controlDt);

      if (this.lastSentX === null || this.lastSentY === null) {
        this.lastSentX = safePos.x;
        this.lastSentY = safePos.y;
      }

      const dx = safePos.x - this.lastSentX;
      const dy = safePos.y - this.lastSentY;
      const dist = Math.sqrt(dx * dx + dy * dy);

      if (dist < 0.001) return;
      if (this.inFlight >= this.maxInFlight) return;

      const vNorm = this.shaper.vel.norm();
      const feedrate = Math.floor(Math.min(99999, Math.max(1000, vNorm * 60)));

      const cmd = `G1 X${safePos.x.toFixed(1)} Y${safePos.y.toFixed(1)} F${feedrate}`;
      this.send(cmd).then(success => {
        if (success) {
          this.lastSentX = safePos.x;
          this.lastSentY = safePos.y;
        }
      });

    }, 1000 / this.controlHz);
  }

  async handleManualGcode(cmd) {
    if (!this.port) return;
    this.isHoming = true;
    
    await this.sendInstant("M410");
    await new Promise(r => setTimeout(r, 50));
    await this.sendInstant("M999");
    await new Promise(r => setTimeout(r, 50));
    this.inFlight = 0;

    await this.sendInstant(cmd);

    if (cmd === "G28") {
      this.printerHomed = true;
      setTimeout(() => {
        this.estimator.pos = new Vec3(this.bedX / 2, this.bedY / 2, 15.0);
        this.estimator.vel = new Vec3(0, 0, 0);
        this.shaper.setPosition(new Vec3(this.bedX / 2, this.bedY / 2, 15.0));
        this.shaper.vel = new Vec3(0, 0, 0);
        this.isHoming = false;
        console.log("[SERIAL] Homing presumed complete.");
      }, 10000);
    } else {
      this.isHoming = false;
    }
  }
}

class LineBreakTransformer {
  constructor() {
    this.chunks = "";
  }
  transform(chunk, controller) {
    this.chunks += chunk;
    const lines = this.chunks.split("\\n");
    this.chunks = lines.pop();
    lines.forEach((line) => controller.enqueue(line));
  }
  flush(controller) {
    if (this.chunks) {
      controller.enqueue(this.chunks);
    }
  }
}
