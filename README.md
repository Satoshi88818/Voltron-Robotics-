# VoltronAI v3.0

> **Full-Stack Humanoid Robot Controller — Real Hardware | 85 kg | Production-Grade**

[![Version](https://img.shields.io/badge/version-3.0-blue.svg)](https://github.com/voltron-ai/voltron)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)
[![ROS](https://img.shields.io/badge/ROS-Noetic%20%7C%20ROS2%20ready-orange.svg)](https://www.ros.org/)
[![License](https://img.shields.io/badge/license-Proprietary-red.svg)]()
[![Status](https://img.shields.io/badge/status-Production--Ready-brightgreen.svg)]()

---

VoltronAI is a full-stack hierarchical controller for a real 85 kg humanoid robot. It combines a leg-aided Unscented Kalman Filter for state estimation, hierarchical reinforcement learning (high-level discrete policy + low-level joint-velocity PPO), centroidal Model Predictive Control backed by CasADi, and a quad-redundant safety architecture. Every component is designed around the constraints of real hardware: deterministic timing, sensor noise, actuator limits, thermal derating, and physical human safety.

---

## Table of Contents

- [What's New in v3.0](#whats-new-in-v30)
- [Architecture](#architecture)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Project Structure](#project-structure)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Modules](#modules)
  - [SafetyManager](#safetymanager)
  - [StateEstimator](#stateestimator)
  - [SensorFusion & GraspDetector](#sensorfusion--graspdetector)
  - [ObservationBuilder](#observationbuilder)
  - [PolicyStack](#policystack)
  - [BehaviorTree](#behaviortree)
  - [CentroidalMPC](#centroidalmpc)
  - [ActuationInterface](#actuationinterface)
  - [TeleopHandler](#teleophandler)
- [Safety System](#safety-system)
- [Deployment](#deployment)
  - [Production (ROS1)](#production-ros1)
  - [Simulation Mode](#simulation-mode)
  - [ONNX Export](#onnx-export)
  - [TensorRT (Jetson Orin)](#tensorrt-jetson-orin)
  - [ROS2 Migration](#ros2-migration)
- [Testing](#testing)
- [Monitoring](#monitoring)
- [Changelog](#changelog)
- [Roadmap](#roadmap)

---

## What's New in v3.0

v3.0 is a major architectural overhaul of v2.5. The key changes are:

**Safety** is now centralised in a dedicated `SafetyManager` class that owns the watchdog, zone enforcement, command validation, GPIO E-stop relay, thermal derating, and the global exception-to-E-stop handler. Nothing bypasses it.

**Quad redundancy** replaces triple redundancy. A shadow PD/impedance controller runs in parallel with the RL policies at every step. If the RL joint targets deviate more than a configurable threshold from the PD targets, the system falls back to PD automatically — no human intervention required.

**`CommandValidator`** checks every joint command for velocity, jerk, torque, and singularity proximity *before* it reaches the hardware, providing a software-level last line of defence independent of the RL policy.

**`BehaviorTree`** replaces the bare state enum switch. Transitions now carry guard functions — for example, the controller cannot move from `WALK` to `LIFT` unless `GRASP_SUCCESS` has been confirmed, preventing dangerous half-executed sequences.

**`ObservationNormalizer`** adds running mean/std normalisation to the observation vector, which is essential for stable deployment of policies trained in simulation.

The **MPC** backend has been upgraded from cvxpy + OSQP to CasADi + IPOPT (with cvxpy retained as a fallback), giving roughly 5–10× lower solve latency on CPU.

The **GraspDetector** now runs a MobileNetV3 ONNX CNN for image-based scoring rather than the binary `image_ok` heuristic.

**LiDAR** processing uses proper angular binning rather than a `[::2]` stride that assumed a fixed sensor layout.

A full **unit test suite** covers the UKF, SafetyManager, CommandValidator, BehaviorTree guards, and the IK safety layer.

---

## Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                        VoltronAI v3.0                              │
│                                                                    │
│  /imu/data ──────► StateEstimator (UKF + F/T contact detection)    │
│  /joint_states ──► joint_pos, joint_vel                            │
│  /scan ──────────► SensorFusion (angular-binned LiDAR, 64 bins)    │
│  /power/status ──► Battery + thermal monitor                       │
│  /wrist_ft ──────► GraspDetector (MobileNetV3 CNN + F/T + pressure)│
│                                                                    │
│              ObservationNormalizer (running mean/std)              │
│                            │                                       │
│                            ▼                                       │
│               ┌────────────────────────┐                           │
│               │   BehaviorTree FSM     │ ← YAML-configured guards  │
│               │   (py_trees backed)    │                           │
│               └────────────┬───────────┘                           │
│                            │ State                                 │
│               ┌────────────▼───────────┐                           │
│               │      PolicyStack       │                           │
│               │  HighLevel (PPO/ONNX)  │                           │
│               │  LowLevel  (PPO/ONNX)  │ ← TensorRT on Jetson Orin │
│               │  ShadowPD  (fallback)  │ ← quad-redundancy         │
│               └────────────┬───────────┘                           │
│                            │ Δq target                             │
│               ┌────────────▼───────────┐                           │
│               │     SafetyManager      │                           │
│               │  CommandValidator      │ ← vel / jerk / singularity│
│               │  Thermal derate        │ ← 70–90°C ramp            │
│               │  Zone enforcement      │ ← 0.8 m / 1.5 m radii    │
│               │  HW watchdog + GPIO    │ ← 500 ms timeout → relay  │
│               └────────────┬───────────┘                           │
│                            │                                       │
│               ┌────────────▼───────────┐                           │
│               │  ActuationInterface    │ ← IK clamp, 200 Hz cap    │
│               └────────────────────────┘                           │
│                            │                                       │
│        /joint_trajectory_controller/command                        │
│                                                                    │
│   CentroidalMPC ── CasADi + IPOPT (20 Hz daemon thread)           │
│   Prometheus ────── :8000/metrics                                  │
└────────────────────────────────────────────────────────────────────┘
```

### Control Hierarchy

| Layer | Frequency | Method | Output |
|-------|-----------|--------|--------|
| High-level policy | 100 Hz | Discrete PPO (ONNX) | State enum |
| Low-level policy | 100 Hz | Velocity PPO (ONNX) | Δq joint velocities |
| Shadow PD | 100 Hz | Impedance PD | Fallback joint targets |
| IK safety layer | 100 Hz | Clamping + null-space | Safe joint positions |
| Centroidal MPC | 20 Hz | CasADi + IPOPT | CoM trajectory |
| State estimator | 200 Hz | Leg-aided UKF | 15-D state vector |

---

## System Requirements

| Requirement | Minimum | Recommended |
|-------------|---------|-------------|
| Python | 3.10 | 3.11 |
| ROS | Noetic (ROS1) | ROS2 Humble (migration path provided) |
| CPU | x86-64, 4 cores | AMD/Intel 8-core, or NVIDIA Jetson Orin |
| RAM | 8 GB | 16 GB |
| OS | Ubuntu 20.04 | Ubuntu 22.04 |
| GPU (optional) | Any CUDA 11+ | Jetson Orin (TensorRT path) |
| PREEMPT_RT | Optional | Strongly recommended for 200 Hz estimator |

---

## Installation

### 1. Clone and install Python dependencies

```bash
git clone https://github.com/voltron-ai/voltron_ai.git
cd voltron_ai
pip install -r requirements.txt
```

### 2. Install ROS packages

```bash
sudo apt install ros-noetic-sensor-msgs \
                 ros-noetic-geometry-msgs \
                 ros-noetic-trajectory-msgs \
                 ros-noetic-std-msgs \
                 ros-noetic-cv-bridge
```

### 3. (Optional) CasADi for fast MPC

```bash
pip install casadi
```

If CasADi is not installed, VoltronAI automatically falls back to `cvxpy` + OSQP. If neither is installed, MPC is disabled and the controller falls back to RL-only output.

### 4. (Optional) GPIO E-stop relay (Raspberry Pi co-processor)

```bash
pip install RPi.GPIO
```

The GPIO pin is configured via `estop_gpio_pin` in your config YAML (default: BCM pin 17). The pin is held HIGH during normal operation and pulled LOW on any E-stop trigger.

### 5. Place trained models

```
models/
├── high_level_policy.onnx   # High-level discrete PPO
├── low_level_policy.onnx    # Low-level velocity PPO
└── grasp_detector.onnx      # MobileNetV3 grasp CNN
```

See [ONNX Export](#onnx-export) to generate these from SB3 checkpoints.

---

## Project Structure

```
voltron_ai/
├── voltron_ai/
│   ├── __init__.py
│   ├── config.py               # OmegaConf config, Pydantic model, checksum verify
│   ├── safety_manager.py       # SafetyManager + CommandValidator + GPIO E-stop
│   ├── state_estimator.py      # Leg-aided UKF, F/T contact detection
│   ├── observation.py          # ObservationNormalizer (Welford), ObservationBuilder
│   ├── policy_stack.py         # PolicyStack, PolicyRunner (ONNX/SB3), ShadowPD
│   ├── behavior_tree.py        # BehaviorTree FSM, State enum, STATE_MAP, guards
│   ├── actuation.py            # ActuationInterface, IKSafetyLayer
│   ├── mpc.py                  # CasADi/IPOPT + cvxpy/OSQP fallback MPC
│   ├── sensor_fusion.py        # SensorFusion (LiDAR bins), GraspDetector (CNN)
│   ├── teleop.py               # Velocity-ramped teleop handler
│   └── voltron_ai.py           # Main VoltronAI controller + entry point
├── tests/
│   ├── test_safety_manager.py
│   ├── test_state_estimator.py
│   ├── test_grasp_detector.py
│   ├── test_command_validator.py
│   └── test_behavior_tree.py
├── models/
│   ├── high_level_policy.onnx
│   ├── low_level_policy.onnx
│   └── grasp_detector.onnx
├── config/
│   ├── base.yaml               # All defaults
│   ├── prod.yaml               # Production overrides + model checksums
│   └── sim.yaml                # Simulation overrides
├── scripts/
│   └── export_onnx.py          # SB3 → ONNX export + SHA-256 output
├── requirements.txt
└── README.md
```

---

## Quick Start

### Production (real hardware)

```bash
# Uses config/prod.yaml overlaid on config/base.yaml
VOLTRON_ENV=prod rosrun voltron_ai voltron_ai.py
```

### Simulation (no ROS required)

```bash
VOLTRON_ENV=sim python -m voltron_ai.voltron_ai
```

### Override a single parameter at runtime

```bash
# Environment variables prefixed VOLTRON_ override any config field
VOLTRON_control_hz=50 VOLTRON_ENV=prod rosrun voltron_ai voltron_ai.py
```

---

## Configuration

VoltronAI uses OmegaConf for layered configuration. The load order is:

```
config/base.yaml  →  config/{VOLTRON_ENV}.yaml  →  VOLTRON_* env vars
```

Each layer merges into the previous; only the fields you specify are overridden.

### Key parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `control_hz` | 100.0 | Main control loop frequency (Hz) |
| `state_estimator_hz` | 200.0 | UKF update frequency (Hz) |
| `max_joint_vel` | 2.0 | Per-joint velocity limit (rad/s) |
| `max_joint_jerk` | 5.0 | Per-joint jerk limit (rad/s³) |
| `watchdog_timeout` | 0.5 | Seconds before watchdog fires E-stop |
| `critical_zone_radius` | 0.8 | Human within this distance → immediate E-stop (m) |
| `warning_zone_radius` | 1.5 | Human within this distance → SAFE_MODE (m) |
| `shadow_pd_deviation_threshold` | 0.3 | Max deviation (rad) before PD fallback triggers |
| `thermal_derate_temp_c` | 70.0 | Temperature at which derating begins (°C) |
| `thermal_max_temp_c` | 90.0 | Temperature at full derate — 50% limits (°C) |
| `mpc_horizon` | 10 | MPC prediction horizon (steps) |
| `mpc_dt` | 0.05 | MPC time step (s) |
| `use_casadi_mpc` | true | Use CasADi (fast). False → cvxpy fallback |
| `use_onnx` | true | Use ONNX Runtime for policy inference |
| `use_tensorrt` | false | Use TensorRT EP (Jetson Orin only) |
| `lidar_num_bins` | 64 | Angular bins for LiDAR observation |
| `obs_norm_momentum` | 0.001 | Running mean/std update rate |
| `rosbag_on_events` | true | Auto-record bag on falls / grasps / near-misses |

### Model checksums (prod.yaml)

Paste SHA-256 hashes from `scripts/export_onnx.py` into `config/prod.yaml` to enable cryptographic verification on model load:

```yaml
model_checksums:
  models/high_level_policy.onnx: "<sha256>"
  models/low_level_policy.onnx:  "<sha256>"
  models/grasp_detector.onnx:    "<sha256>"
```

A checksum mismatch raises `RuntimeError` and prevents startup — ensuring you never accidentally deploy an outdated or corrupted policy.

---

## Modules

### SafetyManager

**File:** `safety_manager.py`

The single safety authority. All other modules call into it; none of them trigger E-stop directly.

**`CommandValidator`** runs before every joint publish and checks:
- Joint velocity (`|Δq / dt| ≤ 1.2 × vel_limit`)
- Joint jerk (estimated from previous command)
- Position limits
- Elbow singularity proximity (< 3° from full extension is rejected)

**Thermal derating** linearly scales velocity and torque limits from 100% at 70°C to 50% at 90°C, based on per-joint driver temperatures from `/joint_drivers/temperature`.

**GPIO E-stop** drives a hardware relay via RPi.GPIO (BCM pin 17 by default). The relay is armed HIGH at startup and pulled LOW on any E-stop event, cutting actuator power independently of software.

**Global exception handler** installs a `sys.excepthook` that calls `emergency_stop()` on any unhandled exception anywhere in the process — preventing silent failures from leaving the robot in an unsafe state.

---

### StateEstimator

**File:** `state_estimator.py`

A 15-dimensional Unscented Kalman Filter:

```
x[0:3]   position (world frame)
x[3:6]   velocity (world frame)
x[6:9]   orientation (roll, pitch, yaw)
x[9:12]  angular velocity
x[12:15] accelerometer bias
```

**v3 improvement:** Contact state is now derived from foot F/T sensors (`/foot_ft/left`, `/foot_ft/right`) rather than an assumed boolean. A ground reaction force above 30 N triggers the zero-velocity update for that foot, improving accuracy on uneven terrain and during partial support phases.

---

### SensorFusion & GraspDetector

**File:** `sensor_fusion.py`

**SensorFusion** handles LiDAR processing. The v2.5 `[::2]` stride has been replaced with proper angular binning: the full angular range of the scan is divided into `lidar_num_bins` equal-width bins, and each bin reports the minimum range within it (most conservative). This is correct for any LiDAR with any angular resolution.

**GraspDetector** fuses three sources:
- Wrist F/T magnitude (35% weight)
- Gripper pressure sensor (35% weight)
- MobileNetV3 ONNX CNN image score (30% weight)

The CNN takes a 224×224 crop from the wrist camera and outputs a binary classifier sigmoid score. If the ONNX model is not found, it falls back to a flat 0.5 heuristic. The final score is a weighted average in [0, 1]; values ≥ 0.5 indicate a successful grasp.

---

### ObservationBuilder

**File:** `observation.py`

Builds the flat observation vector (default dimensionality: 122 for 20-DOF):

| Component | Dim | Source |
|-----------|-----|--------|
| UKF state | 15 | StateEstimator |
| joint_pos | 20 | StateEstimator |
| joint_vel | 20 | StateEstimator |
| LiDAR bins | 64 | SensorFusion |
| Scalars | 3 | `human_dist / 10`, `battery_norm`, `grasp_prob` |

**`ObservationNormalizer`** uses the Welford online algorithm to maintain a running mean and variance. The normalised observation is clipped to [−10, 10]. Statistics are saved to `obs_norm_stats.npz` on shutdown and reloaded at startup, so the normaliser improves continuously across deployments.

---

### PolicyStack

**File:** `policy_stack.py`

Manages all three inference paths:

**High-level policy** (discrete PPO) selects the target `State` enum from the full observation. Outputs an integer index into `STATE_MAP`.

**Low-level policy** (velocity PPO) outputs per-joint velocity deltas Δq scaled by `max_joint_vel`. These are integrated over `dt` to produce a joint position target.

**Shadow PD controller** runs an impedance-style PD law toward the nominal upright pose at every tick, entirely independently of the RL policies. If the RL joint target deviates from the PD target by more than `shadow_pd_deviation_threshold` (default: 0.3 rad) in any joint, the PD output is used instead. This is the fourth redundancy layer after watchdog, zone enforcement, and CommandValidator.

Both RL policies support ONNX Runtime (CPU or CUDA) and TensorRT (Jetson Orin) inference paths. SB3 is retained as a fallback if ONNX models are absent.

---

### BehaviorTree

**File:** `behavior_tree.py`

A guarded finite state machine backed by `py_trees` when available, with a lightweight dict-based FSM fallback.

**States:**

| Index | State | Description |
|-------|-------|-------------|
| 0 | `IDLE` | Stationary, ready |
| 1 | `WALK` | Bipedal locomotion |
| 2 | `GRASP` | Approaching and gripping |
| 3 | `GRASP_SUCCESS` | Object securely held |
| 4 | `LIFT` | Raising held object |
| 5 | `RECOVER` | Fall recovery |
| 6 | `EMERGENCY_STOP` | All motion halted |
| 7 | `HUMAN_INTERACTION` | Proximity interaction mode |
| 8 | `RETURN_TO_BASE` | Autonomous return |
| 9 | `NOD` | Acknowledgement gesture |
| 10 | `SAFE_MODE` | Reduced-speed mode |
| 11 | `TELEOP` | Human-in-the-loop control |

**Guarded transitions (selected):**

| From | To | Guard |
|------|----|-------|
| `WALK` | `LIFT` | `grasp_success == True` |
| `GRASP` | `LIFT` | `grasp_success == True` |
| `IDLE` | `WALK` | `battery_ok == True` |
| `RECOVER` | `WALK` | `walk_stable == True` |
| Any | `EMERGENCY_STOP` | Always allowed |
| Any | `SAFE_MODE` | Always allowed |
| Any | `TELEOP` | Always allowed |

`STATE_MAP` is an explicit `Dict[int, State]` — the integer assignments are fixed and independent of Python enum definition order.

---

### CentroidalMPC

**File:** `mpc.py`

Centroidal dynamics MPC running in a daemon thread at 20 Hz. The main 100 Hz loop calls `request_solve(state, ref_traj)` without blocking; the latest solution is available via `mpc.last_solution`.

**CasADi path (default):** NLP formulated with CasADi and solved with IPOPT. Significantly faster than cvxpy on CPU — typical solve times are 2–5 ms for a 10-step horizon.

**cvxpy fallback:** The v2.5 OSQP-backed formulation is retained and activated automatically if CasADi is not installed.

The MPC optimises CoM position and velocity over a receding horizon, subject to friction cone constraints (μ = 0.7), ground reaction force bounds (50–1500 N per foot), and an initial condition pinning constraint.

---

### ActuationInterface

**File:** `actuation.py`

Publishes `JointTrajectory` messages to `/joint_trajectory_controller/command`.

- Internally rate-limited to 200 Hz to prevent controller flooding.
- Every command passes through `IKSafetyLayer` which clamps positions to hardware limits and velocity to `vel_limits × thermal_factor × dt`.
- On publish failure, falls back to a safe all-zeros pose without waiting for the next control tick.
- In simulation mode (`sim_mode=True`), stores the last command in `_last_sim_command` for external test harnesses to read.

---

### TeleopHandler

**File:** `teleop.py`

Subscribes to `/teleop/enable` (Bool) and `/teleop/joint_cmd` (JointState).

Operator commands are smoothed through a first-order exponential ramp with a 200 ms time constant, preventing sudden jumps when the operator issues step inputs. All teleop commands pass through the same `
