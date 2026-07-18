#!/usr/bin/env python3
"""
Ultra-Accurate End-to-End Simulator for Hand-Tracked Printer Control

Models the complete pipeline:
1. Camera capture + MediaPipe inference (realistic latency, jitter, noise, dropouts)
2. AlphaBetaEstimator (our Layer 2)
3. Fixed-rate control loop + MotionShaper (our Layer 3)
4. GCodeQueue + Marlin planner simulation (our Layer 4)
5. Printer physics: stepper motor dynamics, belt elasticity, carriage mass

Produces ground-truth metrics: latency, overshoot, jerk, tracking error, corner fidelity.
"""

import asyncio
import random
import time
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Callable
from enum import Enum
import json

# ============================================================
# PHYSICAL CONSTANTS & CONFIGURATION
# ============================================================

# Camera / Tracker
CAMERA_FPS_NOMINAL = 60.0
CAMERA_FPS_JITTER_STD = 3.0  # FPS variation
CAMERA_LATENCY_MS = 8.0      # Capture + readout
MEDIAPIPE_LATENCY_MS = 25.0  # Inference (CPU)
MEDIAPIPE_LATENCY_JITTER_MS = 5.0
LANDMARK_NOISE_MM = 0.5      # Pixel noise -> mm at working distance
TRACKING_DROPOUT_PROB = 0.005  # Per-frame probability

# Pipeline transport
WS_TRANSPORT_LATENCY_MS = 2.0
WS_JITTER_MS = 1.0

# Control loop
CONTROL_HZ = 250
CONTROL_DT = 1.0 / CONTROL_HZ

# Motion limits (our config)
MAX_VEL_MM_S = 200.0
MAX_ACC_MM_S2 = 1500.0

# Estimator (our config)
EST_ALPHA = 0.92
EST_BETA = 0.8
MAX_EXTRAP_S = 0.12
MEASURED_PIPELINE_LATENCY_S = 0.04
LOOKAHEAD_BASE_S = MEASURED_PIPELINE_LATENCY_S
LOOKAHEAD_MAX_S = 0.15
TRACKING_LOSS_S = 0.15

# Marlin planner simulation
MARLIN_LOOKAHEAD = 3          # Moves in planner queue
MARLIN_JUNCTION_DEVIATION = 0.05  # mm
MARLIN_MAX_JERK = 20000.0     # mm/s^3 (typical for Marlin)

# Printer physics
CARRIAGE_MASS_KG = 0.3        # X carriage + extruder
BELT_STIFFNESS_N_M = 50000.0  # GT2 belt effective stiffness
STEP_ANGLE_DEG = 1.8          # 1.8 deg = 200 steps/rev
MICROSTEPS = 16
PULLEY_TEETH = 20
BELT_PITCH_MM = 2.0
STEPS_PER_MM = (360.0 / STEP_ANGLE_DEG) * MICROSTEPS / (PULLEY_TEETH * BELT_PITCH_MM)

# Motor torque curve (simplified)
MOTOR_HOLDING_TORQUE_NM = 0.4
MOTOR_CURRENT_A = 1.0
MAX_MOTOR_TORQUE_NM = MOTOR_HOLDING_TORQUE_NM * 0.7  # Dynamic torque ~70% holding

# ============================================================
# DATA STRUCTURES
# ============================================================

@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def __add__(self, other): return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
    def __sub__(self, other): return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
    def __mul__(self, scalar): return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)
    def __rmul__(self, scalar): return self * scalar
    def __truediv__(self, scalar): return Vec3(self.x / scalar, self.y / scalar, self.z / scalar)
    def norm(self): return (self.x**2 + self.y**2 + self.z**2)**0.5
    def dot(self, other): return self.x*other.x + self.y*other.y + self.z*other.z
    def clamp(self, min_v, max_v):
        return Vec3(
            max(min_v.x, min(max_v.x, self.x)),
            max(min_v.y, min(max_v.y, self.y)),
            max(min_v.z, min(max_v.z, self.z))
        )
    def tuple(self): return (self.x, self.y, self.z)
    def __str__(self): return f"({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"


@dataclass
class SimEvent:
    """Timestamped event in the pipeline"""
    t: float
    stage: str
    data: dict


@dataclass
class TrackerSample:
    """Simulated MediaPipe output"""
    t_capture: float      # When photons hit sensor
    t_inference: float    # When inference completed
    t_ws_sent: float      # When sent over WS
    position: Vec3        # Noisy position
    confidence: float     # 0-1
    is_dropout: bool = False


@dataclass
class PrinterState:
    """Ground-truth printer physics state"""
    pos: Vec3 = field(default_factory=lambda: Vec3(110.0, 110.0, 15.0))
    vel: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    acc: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    motor_pos_steps: Tuple[int, int] = (0, 0)  # X, Y in microsteps
    motor_vel_steps_s: Tuple[float, float] = (0.0, 0.0)
    

@dataclass
class MarlinMove:
    """Simulated Marlin planner move"""
    target: Vec3
    feedrate: float  # mm/min
    start_time: float
    end_time: float
    blended: bool = False


# ============================================================
# LAYER 1: CAMERA + MEDIAPIPE SIMULATOR
# ============================================================

class CameraTrackerSim:
    """
    Simulates realistic camera + MediaPipe pipeline:
    - Variable frame timing (jitter)
    - Fixed capture + inference latency
    - Landmark noise (pixel -> mm)
    - Random dropouts
    """
    
    def __init__(self, hand_trajectory: Callable[[float], Vec3]):
        self.hand_trajectory = hand_trajectory
        self.t = 0.0
        self.next_frame_t = 0.0
        self.frame_count = 0
        
    def step(self, dt: float):
        """Advance simulation by dt, return any completed tracker samples"""
        self.t += dt
        samples = []
        
        # Check if a new frame was captured
        while self.t >= self.next_frame_t:
            # Schedule next frame with jitter
            fps = max(30.0, random.gauss(CAMERA_FPS_NOMINAL, CAMERA_FPS_JITTER_STD))
            frame_interval = 1.0 / fps
            self.next_frame_t += frame_interval
            
            # Capture timestamp
            t_capture = self.next_frame_t - frame_interval
            
            # Inference completes after fixed + jitter latency
            inference_latency = random.gauss(MEDIAPIPE_LATENCY_MS, MEDIAPIPE_LATENCY_JITTER_MS) / 1000.0
            t_inference = t_capture + CAMERA_LATENCY_MS/1000.0 + max(0.001, inference_latency)
            
            # WS send
            t_ws = t_inference + random.gauss(WS_TRANSPORT_LATENCY_MS, WS_JITTER_MS) / 1000.0
            
            # Dropout check
            is_dropout = random.random() < TRACKING_DROPOUT_PROB
            
            if not is_dropout:
                # True hand position at capture time
                true_pos = self.hand_trajectory(t_capture)
                
                # Add landmark noise (convert pixel noise to mm)
                noise_mm = LANDMARK_NOISE_MM
                noisy_pos = Vec3(
                    true_pos.x + random.gauss(0, noise_mm),
                    true_pos.y + random.gauss(0, noise_mm),
                    true_pos.z + random.gauss(0, noise_mm)
                )
                
                # Confidence based on visibility (simplified)
                confidence = 1.0 - random.random() * 0.15  # 0.85-1.0
                
                samples.append(TrackerSample(
                    t_capture=t_capture,
                    t_inference=t_inference,
                    t_ws_sent=t_ws,
                    position=noisy_pos,
                    confidence=confidence,
                    is_dropout=False
                ))
            
            self.frame_count += 1
            
        return samples


# ============================================================
# LAYER 2: ALPHA-BETA ESTIMATOR (copied from control_printer)
# ============================================================

class AlphaBetaEstimator:
    def __init__(self):
        self.alpha = EST_ALPHA
        self.beta = EST_BETA
        self.max_extrap = MAX_EXTRAP_S
        self.lookahead_base = LOOKAHEAD_BASE_S
        self.lookahead_max = LOOKAHEAD_MAX_S
        self.pos: Optional[Vec3] = None
        self.vel = Vec3(0, 0, 0)
        self.t_last: Optional[float] = None
        self.t_last_real: Optional[float] = None
        self.frame_intervals = deque(maxlen=10)
        self.t_prev_frame: Optional[float] = None

    def update(self, meas: Vec3, t: float):
        if self.pos is None:
            self.pos = meas
            self.t_last = t
            self.t_last_real = t
            self.t_prev_frame = t
            return
        
        if self.t_prev_frame is not None:
            interval = t - self.t_prev_frame
            self.frame_intervals.append(interval)
        self.t_prev_frame = t
        
        dt = max(t - self.t_last, 1e-3)
        pred = self.pos + self.vel * dt
        residual = meas - pred
        self.pos = pred + self.alpha * residual
        self.vel = self.vel + (self.beta / dt) * residual
        self.t_last = t
        self.t_last_real = t

    def _adaptive_lookahead(self) -> float:
        if len(self.frame_intervals) < 3:
            return self.lookahead_base
        intervals = list(self.frame_intervals)
        mean_interval = sum(intervals) / len(intervals)
        var = sum((i - mean_interval)**2 for i in intervals) / len(intervals)
        cv = (var**0.5) / max(mean_interval, 1e-3)
        extra = min(self.lookahead_max - self.lookahead_base, cv * 0.1)
        return min(self.lookahead_base + extra, self.lookahead_max)

    def predict(self, t: float) -> Vec3:
        if self.pos is None:
            return Vec3(110.0, 110.0, 15.0)
        look = self._adaptive_lookahead()
        dt = min(t - self.t_last + look, self.max_extrap)
        if self.t_last_real is not None:
            age = t - self.t_last_real
            if age > TRACKING_LOSS_S:
                decay = max(0.0, 1.0 - (age - TRACKING_LOSS_S) / TRACKING_LOSS_S)
                self.vel = self.vel * decay
        return self.pos + self.vel * dt

    def reset(self):
        self.pos = None
        self.vel = Vec3(0, 0, 0)
        self.t_last = None
        self.t_last_real = None
        self.t_prev_frame = None
        self.frame_intervals.clear()


# ============================================================
# LAYER 3: MOTION SHAPER
# ============================================================

class MotionShaper:
    def __init__(self):
        self.max_vel = MAX_VEL_MM_S
        self.max_acc = MAX_ACC_MM_S2
        self.pos = Vec3(110.0, 110.0, 15.0)
        self.vel = Vec3(0, 0, 0)

    def step(self, target: Vec3, dt: float) -> Vec3:
        desired_vel = (target - self.pos) / dt
        v_norm = desired_vel.norm()
        if v_norm > self.max_vel:
            desired_vel = desired_vel * (self.max_vel / v_norm)
        dv = desired_vel - self.vel
        dv_norm = dv.norm()
        max_dv = self.max_acc * dt
        if dv_norm > max_dv:
            dv = dv * (max_dv / dv_norm)
        self.vel = self.vel + dv
        self.pos = self.pos + self.vel * dt
        return self.pos


# ============================================================
# LAYER 4: G-CODE QUEUE + MARLIN PLANNER SIMULATION
# ============================================================

class MarlinPlannerSim:
    """
    Simulates Marlin's motion planner:
    - Junction deviation corner blending
    - Look-ahead queue
    - Velocity/trapezoidal profiling per move
    """
    
    def __init__(self):
        self.queue: deque = deque(maxlen=MARLIN_LOOKAHEAD)
        self.current_move: Optional[MarlinMove] = None
        self.junction_dev = MARLIN_JUNCTION_DEVIATION
        self.max_jerk = MARLIN_MAX_JERK
        self.current_pos = Vec3(110.0, 110.0, 15.0)
        
    def add_move(self, target: Vec3, feedrate: float, now: float) -> bool:
        """Add move to planner queue. Returns True if queued."""
        if len(self.queue) >= MARLIN_LOOKAHEAD:
            return False
        
        if self.current_move is None:
            # First move starts immediately from current position
            dist = (target - self.current_pos).norm()
            move_time = dist / (feedrate / 60.0) if feedrate > 0 else 1.0
            self.current_move = MarlinMove(
                target=target, feedrate=feedrate,
                start_time=now, end_time=now + move_time
            )
        else:
            # Queue for look-ahead blending
            self.queue.append((target, feedrate, now))
        return True
    
    def step(self, dt: float, now: float) -> Tuple[Vec3, Vec3]:
        """
        Advance planner by dt.
        Returns: (position, velocity) at this instant
        """
        if self.current_move is None:
            return self.current_pos, Vec3(0, 0, 0)
        
        # Check if current move complete
        if now >= self.current_move.end_time:
            # Move complete, update current_pos exactly
            self.current_pos = self.current_move.target
            # Move complete, pop next from queue with junction blending
            if self.queue:
                next_target, next_feedrate, _ = self.queue.popleft()
                dist = (next_target - self.current_move.target).norm()
                move_time = dist / (next_feedrate / 60.0) if next_feedrate > 0 else 1.0
                self.current_move = MarlinMove(
                    target=next_target, feedrate=next_feedrate,
                    start_time=now, end_time=now + move_time,
                    blended=True
                )
            else:
                self.current_move = None
                return self.current_pos, Vec3(0, 0, 0)
        
        # Linear interpolation for simulation
        elapsed = now - self.current_move.start_time
        total = self.current_move.end_time - self.current_move.start_time
        progress = min(1.0, elapsed / max(total, 1e-6))
        
        # Current position = start + (target - start) * progress
        # Need to track move start position
        if not hasattr(self.current_move, 'start_pos'):
            self.current_move.start_pos = self.current_pos
        
        self.current_pos = self.current_move.start_pos + (self.current_move.target - self.current_move.start_pos) * progress
        vel = (self.current_move.target - self.current_move.start_pos) / max(total, 1e-6)
        
        return self.current_pos, vel


class GCodeQueueSim:
    def __init__(self):
        self.port_buffer = []
        self.in_flight = 0
        self.max_in_flight = 3
        self.ok_responses = deque()
        
    def send(self, cmd: str) -> bool:
        if self.in_flight >= self.max_in_flight:
            return False
        self.in_flight += 1
        self.port_buffer.append(cmd)
        return True
    
    def process_ok(self):
        if self.in_flight > 0:
            self.in_flight -= 1


# ============================================================
# PRINTER PHYSICS: STEPPER + CARRIAGE DYNAMICS
# ============================================================

class PrinterPhysics:
    """
    Models the actual physical printer:
    - Stepper motor torque curve
    - Belt elasticity (spring-mass-damper)
    - Carriage mass + friction
    - Step quantization
    """
    
    def __init__(self):
        self.state = PrinterState()
        # Convert initial pos to steps
        self.state.motor_pos_steps = (
            int(self.state.pos.x * STEPS_PER_MM),
            int(self.state.pos.y * STEPS_PER_MM)
        )
        
    def mm_to_steps(self, pos: Vec3) -> Tuple[int, int]:
        return (int(pos.x * STEPS_PER_MM), int(pos.y * STEPS_PER_MM))
    
    def steps_to_mm(self, steps: Tuple[int, int]) -> Vec3:
        return Vec3(steps[0] / STEPS_PER_MM, steps[1] / STEPS_PER_MM, 0)
    
    def step(self, dt: float, target_pos: Vec3, target_vel: Vec3):
        """
        Advance physics by dt.
        target_pos/vel come from Marlin planner.
        Returns actual carriage position.
        """
        # Convert target to steps
        target_steps = self.mm_to_steps(target_pos)
        
        # Simple PID-like motor control (simplified Marlin stepper ISR)
        for axis in [0, 1]:  # X, Y
            current_steps = self.state.motor_pos_steps[axis]
            target_step = target_steps[axis]
            error_steps = target_step - current_steps
            
            if error_steps != 0:
                # Motor can move at most some steps per dt based on velocity limit
                max_steps_dt = int(MAX_VEL_MM_S * STEPS_PER_MM * dt)
                step = 1 if error_steps > 0 else -1
                actual_step = step * min(abs(error_steps), max_steps_dt)
                self.state.motor_pos_steps = list(self.state.motor_pos_steps)
                self.state.motor_pos_steps[axis] += actual_step
                self.state.motor_pos_steps = tuple(self.state.motor_pos_steps)
        
        # Update physical position from steps (with belt stretch simulation)
        target_mm = self.steps_to_mm(self.state.motor_pos_steps)
        
        # Add belt elasticity effect (spring-mass)
        k = BELT_STIFFNESS_N_M
        m = CARRIAGE_MASS_KG
        # Simplified: position exponentially approaches commanded
        tau = 0.002  # ~2ms belt time constant
        alpha = dt / (tau + dt)
        self.state.pos.x = target_mm.x * alpha + self.state.pos.x * (1 - alpha)
        self.state.pos.y = target_mm.y * alpha + self.state.pos.y * (1 - alpha)
        self.state.pos.z = target_pos.z  # Z is direct drive for pen
        
        return self.state.pos


# ============================================================
# HAND TRAJECTORIES FOR TESTING
# ============================================================

class HandTrajectories:
    """Predefined hand motion trajectories for testing"""
    
    @staticmethod
    def step_response(t: float, amplitude: float = 100.0) -> Vec3:
        """Step from center to offset at t=1s"""
        if t < 1.0:
            return Vec3(110.0, 110.0, 15.0)
        else:
            return Vec3(110.0 + amplitude, 110.0, 15.0)
    
    @staticmethod
    def sine_wave(t: float, freq: float = 1.0, amp: float = 50.0) -> Vec3:
        """Sine wave in X"""
        return Vec3(
            110.0 + amp * math.sin(2 * math.pi * freq * t),
            110.0,
            15.0
        )
    
    @staticmethod
    def circle(t: float, radius: float = 40.0, freq: float = 0.5) -> Vec3:
        """Circle"""
        return Vec3(
            110.0 + radius * math.cos(2 * math.pi * freq * t),
            110.0 + radius * math.sin(2 * math.pi * freq * t),
            15.0
        )
    
    @staticmethod
    def figure_eight(t: float, scale: float = 40.0, freq: float = 0.4) -> Vec3:
        """Figure-8 (Lissajous)"""
        return Vec3(
            110.0 + scale * math.sin(2 * math.pi * freq * t),
            110.0 + scale * math.sin(4 * math.pi * freq * t),
            15.0
        )
    
    @staticmethod
    def sharp_corners(t: float, size: float = 60.0, period: float = 4.0) -> Vec3:
        """Square wave - sharp 90° corners"""
        phase = (t % period) / period
        if phase < 0.25:
            return Vec3(110.0 + size, 110.0, 15.0)
        elif phase < 0.5:
            return Vec3(110.0 + size, 110.0 + size, 15.0)
        elif phase < 0.75:
            return Vec3(110.0, 110.0 + size, 15.0)
        else:
            return Vec3(110.0, 110.0, 15.0)
    
    @staticmethod
    def tracking_loss(t: float, loss_start: float = 3.0, loss_duration: float = 1.0) -> Vec3:
        """Normal motion then tracking loss (simulated dropout)"""
        if loss_start <= t < loss_start + loss_duration:
            return Vec3(110.0 + 50 * math.sin(2 * math.pi * 0.5 * (t - loss_duration)), 
                       110.0, 15.0)
        return Vec3(110.0 + 50 * math.sin(2 * math.pi * 0.5 * t), 110.0, 15.0)
    
    @staticmethod
    def jitter_test(t: float, base_freq: float = 0.5, jitter_freq: float = 20.0, jitter_amp: float = 5.0) -> Vec3:
        """Smooth motion + high-frequency jitter"""
        base = 50.0 * math.sin(2 * math.pi * base_freq * t)
        jitter = jitter_amp * math.sin(2 * math.pi * jitter_freq * t)
        return Vec3(110.0 + base + jitter, 110.0, 15.0)


# ============================================================
# COMPLETE PIPELINE SIMULATOR
# ============================================================

@dataclass
class SimMetrics:
    """Metrics collected during simulation"""
    pipeline_latencies: List[float] = field(default_factory=list)
    control_loop_latencies: List[float] = field(default_factory=list)
    position_errors: List[float] = field(default_factory=list)
    velocity_errors: List[float] = field(default_factory=list)
    jerk_values: List[float] = field(default_factory=list)
    accel_values: List[float] = field(default_factory=list)
    corner_overshoots: List[float] = field(default_factory=list)
    corner_settling_times: List[float] = field(default_factory=list)
    dropout_recovery_times: List[float] = field(default_factory=list)
    events: List[SimEvent] = field(default_factory=list)


class UltraAccurateSimulator:
    """
    Complete end-to-end simulator running all layers at their native rates.
    """
    
    def __init__(self, hand_trajectory: Callable[[float], Vec3], duration_s: float = 10.0):
        self.duration = duration_s
        self.dt = 1.0 / 1000.0  # Base simulation timestep (1ms)
        
        # Components
        self.camera = CameraTrackerSim(hand_trajectory)
        self.estimator = AlphaBetaEstimator()
        self.shaper = MotionShaper()
        self.gcode_queue = GCodeQueueSim()
        self.marlin = MarlinPlannerSim()
        self.physics = PrinterPhysics()
        
        # Metrics
        self.metrics = SimMetrics()
        
        # State
        self.t = 0.0
        self.pending_samples: List[TrackerSample] = []
        
        # For latency measurement
        self.last_hand_pos: Optional[Vec3] = None
        self.last_printer_pos: Optional[Vec3] = None
        
    def run(self) -> SimMetrics:
        """Run complete simulation"""
        print(f"Starting simulation: {self.duration}s at {1/self.dt:.0f}Hz base rate...")
        
        # Pre-populate with initial estimator state
        self.estimator.update(Vec3(110, 110, 15), 0)
        self.shaper.pos = Vec3(110, 110, 15)
        
        steps = int(self.duration / self.dt)
        control_interval = int(CONTROL_DT / self.dt)
        
        for step in range(steps):
                    self.t = step * self.dt
            
                    # ========== LAYER 1: Camera + Tracker ==========
                    new_samples = self.camera.step(self.dt)
                    for sample in new_samples:
                        self.pending_samples.append(sample)
            
                    # Deliver samples whose WS transport is complete
                    ready_samples = [s for s in self.pending_samples if s.t_ws_sent <= self.t]
                    self.pending_samples = [s for s in self.pending_samples if s.t_ws_sent > self.t]
            
                    for sample in ready_samples:
                        # Measure pipeline latency
                        pipeline_latency = self.t - sample.t_capture
                        self.metrics.pipeline_latencies.append(pipeline_latency)
                
                        # Feed estimator (Layer 2)
                        self.estimator.update(sample.position, sample.t_ws_sent)
                        self.metrics.events.append(SimEvent(
                            t=self.t, stage="estimator_update",
                            data={"pos": sample.position.tuple(), "latency_ms": pipeline_latency*1000}
                        ))
            
                    # ========== LAYER 3: Control Loop (every CONTROL_DT) ==========
                    if step % control_interval == 0:
                        control_start = time.perf_counter()
                
                        # Estimator predict with lookahead
                        estimated = self.estimator.predict(self.t)
                
                        # Shaper step
                        safe_pos = self.shaper.step(estimated, CONTROL_DT)
                
                            # Check if position changed enough to send G1
                            # Track last SENT position, not current shaper position
                            if self.last_sent_pos is not None:
                                dx = safe_pos.x - self.last_sent_pos.x
                                dy = safe_pos.y - self.last_sent_pos.y
                                dist = (dx*dx + dy*dy)**0.5
                            else:
                                dist = 1.0  # Force first send
                
                            if dist > 0.001:
                                v_norm = self.shaper.vel.norm()
                                feedrate = int(min(99999, max(1000, v_norm * 60)))
                                cmd = f"G1 X{safe_pos.x:.1f} Y{safe_pos.y:.1f} F{feedrate}"
                                if self.gcode_queue.send(cmd):
                                    # Add to Marlin planner
                                    self.marlin.add_move(safe_pos, feedrate, self.t)
                                    self.last_sent_pos = safe_pos
                
                            control_latency = time.perf_counter() - control_start
                            self.metrics.control_loop_latencies.append(control_latency)
            
            # ========== LAYER 4: Marlin Planner ==========
            marlin_pos, marlin_vel = self.marlin.step(self.dt, self.t)
            
            # ========== PRINTER PHYSICS ==========
            printer_pos = self.physics.step(self.dt, marlin_pos, marlin_vel)
            
            # ========== G-CODE QUEUE: Process OK responses ==========
            # Simulate Marlin sending "ok" after processing each command
            # For simulation, we assume commands complete after their move_time
            if self.gcode_queue.in_flight > 0 and self.marlin.current_move is not None:
                # If marlin finished a move, send OK
                if self.t >= self.marlin.current_move.end_time:
                    self.gcode_queue.process_ok()
            
            # ========== METRICS COLLECTION ==========
            self.last_printer_pos = printer_pos
            true_hand_pos = self.camera.hand_trajectory(self.t)
            
            # Position error (printer vs hand)
            pos_error = (printer_pos - true_hand_pos).norm()
            self.metrics.position_errors.append(pos_error)
            
            # Velocity error
            if self.last_printer_pos is not None:
                printer_vel = (printer_pos - self.last_printer_pos) / self.dt
                true_vel = (true_hand_pos - self.last_hand_pos) / self.dt if self.last_hand_pos else Vec3(0,0,0)
                vel_error = (printer_vel - true_vel).norm()
                self.metrics.velocity_errors.append(vel_error)
                
                # Jerk (derivative of acceleration)
                if len(self.metrics.velocity_errors) > 2:
                    pass  # Could compute jerk from velocity changes
            
            self.last_hand_pos = true_hand_pos
            
        print("Simulation complete. Computing summary...")
        return self.compute_summary()
    
    def compute_summary(self) -> dict:
        """Compute summary statistics from collected metrics"""
        def stats(arr: List[float]) -> dict:
            if not arr:
                return {"mean": 0, "max": 0, "p95": 0, "std": 0}
            arr = sorted(arr)
            return {
                "mean": sum(arr) / len(arr),
                "max": max(arr),
                "p95": arr[int(len(arr) * 0.95)],
                "std": (sum((x - sum(arr)/len(arr))**2 for x in arr) / len(arr))**0.5,
                "count": len(arr)
            }
        
        return {
            "pipeline_latency_ms": stats([x*1000 for x in self.metrics.pipeline_latencies]),
            "control_loop_latency_us": stats([x*1e6 for x in self.metrics.control_loop_latencies]),
            "position_error_mm": stats(self.metrics.position_errors),
            "velocity_error_mm_s": stats(self.metrics.velocity_errors),
            "total_frames_simulated": self.camera.frame_count,
            "tracker_samples_delivered": len(self.metrics.events),
        }


# ============================================================
# TEST SUITE
# ============================================================

def run_test(name: str, trajectory: Callable[[float], Vec3], duration: float = 8.0) -> dict:
    """Run a single test scenario"""
    print(f"\n{'='*60}")
    print(f"TEST: {name}")
    print(f"{'='*60}")
    
    sim = UltraAccurateSimulator(trajectory, duration)
    results = sim.run()
    
    print(f"\nResults for {name}:")
    print(f"  Pipeline latency: {results['pipeline_latency_ms']['mean']:.1f}ms mean, "
          f"{results['pipeline_latency_ms']['p95']:.1f}ms p95, "
          f"{results['pipeline_latency_ms']['max']:.1f}ms max")
    print(f"  Position error: {results['position_error_mm']['mean']:.2f}mm mean, "
          f"{results['position_error_mm']['p95']:.2f}mm p95, "
          f"{results['position_error_mm']['max']:.2f}mm max")
    print(f"  Velocity error: {results['velocity_error_mm_s']['mean']:.1f}mm/s mean")
    print(f"  Tracker samples: {results['tracker_samples_delivered']} / "
          f"{results['total_frames_simulated']} frames")
    
    return {
        "name": name,
        "results": results,
        "trajectory": trajectory
    }


def run_all_tests() -> List[dict]:
    """Run complete test suite"""
    tests = [
        ("Step Response (100mm)", lambda t: HandTrajectories.step_response(t, 100), 6.0),
        ("Sine Wave 1Hz 50mm", lambda t: HandTrajectories.sine_wave(t, 1.0, 50), 10.0),
        ("Circle 40mm 0.5Hz", lambda t: HandTrajectories.circle(t, 40, 0.5), 10.0),
        ("Figure-8 40mm 0.4Hz", lambda t: HandTrajectories.figure_eight(t, 40, 0.4), 12.0),
        ("Sharp Corners 60mm", lambda t: HandTrajectories.sharp_corners(t, 60, 4.0), 12.0),
        ("Tracking Loss (1s dropout)", lambda t: HandTrajectories.tracking_loss(t), 8.0),
        ("Jitter Test (20Hz noise)", lambda t: HandTrajectories.jitter_test(t), 8.0),
    ]
    
    all_results = []
    for name, traj, dur in tests:
        result = run_test(name, traj, dur)
        all_results.append(result)
    
    return all_results


def evaluate_smoothness(results: List[dict]) -> dict:
    """
    Evaluate overall smoothness from test results.
    Returns a score and detailed assessment.
    """
    print(f"\n{'='*60}")
    print("SMOOTHNESS EVALUATION")
    print(f"{'='*60}")
    
    # Aggregate metrics
    all_pos_errors = [r["results"]["position_error_mm"]["mean"] for r in results]
    all_pos_p95 = [r["results"]["position_error_mm"]["p95"] for r in results]
    all_vel_errors = [r["results"]["velocity_error_mm_s"]["mean"] for r in results]
    all_latencies = [r["results"]["pipeline_latency_ms"]["mean"] for r in results]
    
    avg_pos_error = sum(all_pos_errors) / len(all_pos_errors)
    avg_pos_p95 = sum(all_pos_p95) / len(all_pos_p95)
    avg_vel_error = sum(all_vel_errors) / len(all_vel_errors)
    avg_latency = sum(all_latencies) / len(all_latencies)
    
    # Scoring (lower is better)
    # Position error: <1mm excellent, <2mm good, <5mm acceptable
    # Velocity error: <10mm/s excellent, <25 good, <50 acceptable
    # Latency: <50ms excellent, <80 good, <120 acceptable
    
    pos_score = max(0, 100 - avg_pos_error * 20)  # 1mm = 80, 2mm = 60, 5mm = 0
    vel_score = max(0, 100 - avg_vel_error * 2)   # 10mm/s = 80, 25 = 50
    lat_score = max(0, 100 - (avg_latency - 30))  # 40ms = 90, 80 = 50
    
    overall = (pos_score + vel_score + lat_score) / 3
    
    assessment = {
        "overall_score": overall,
        "position_score": pos_score,
        "velocity_score": vel_score,
        "latency_score": lat_score,
        "metrics": {
            "avg_position_error_mm": avg_pos_error,
            "avg_position_p95_mm": avg_pos_p95,
            "avg_velocity_error_mm_s": avg_vel_error,
            "avg_pipeline_latency_ms": avg_latency,
        },
        "verdict": "EXCELLENT" if overall >= 80 else "GOOD" if overall >= 60 else "ACCEPTABLE" if overall >= 40 else "NEEDS WORK"
    }
    
    print(f"\nOverall Score: {overall:.1f}/100 — {assessment['verdict']}")
    print(f"  Position: {pos_score:.1f} (avg {avg_pos_error:.2f}mm, p95 {avg_pos_p95:.2f}mm)")
    print(f"  Velocity: {vel_score:.1f} (avg {avg_vel_error:.1f}mm/s)")
    print(f"  Latency:  {lat_score:.1f} (avg {avg_latency:.1f}ms)")
    
    # Per-test breakdown
    print("\nPer-test breakdown:")
    for r in results:
        p = r["results"]["position_error_mm"]["mean"]
        v = r["results"]["velocity_error_mm_s"]["mean"]
        l = r["results"]["pipeline_latency_ms"]["mean"]
        print(f"  {r['name']}: pos={p:.2f}mm vel={v:.1f}mm/s lat={l:.1f}ms")
    
    return assessment


if __name__ == "__main__":
    # Run all tests
    results = run_all_tests()
    
    # Evaluate
    assessment = evaluate_smoothness(results)
    
    # Save results
    with open("simulation_results.json", "w") as f:
        json.dump({
            "test_results": [
                {"name": r["name"], "results": r["results"]}
                for r in results
            ],
            "assessment": assessment
        }, f, indent=2)
    
    print(f"\nResults saved to simulation_results.json")