# rebot_control

**[中文文档](README_zh.md)**

MIT position control for the reBot B601-RS 6-axis robot arm (Python). It drives RobStride motors over a CAN bus via MotorBridge.

## Warning

> **This project only exposes the robot control interface; it does not implement software joint or workspace limits.** Target angles you set are sent to the motors without software-side range checks.
>
> **Keep the arm within roughly 70% of its workspace during use.** Avoid joint limits and singular configurations. Operating near mechanical limits can cause collisions, damage, or loss of control. Implement limits in your own application layer, or strictly constrain target angles and motion range.

## Hardware

- Robot arm: reBot B601-RS (6 joints + end-effector gripper)
- Motors: J1-J3 are RS06, J4-J6 are RS00, gripper CAN ID **7** (RS00)
- Communication: CAN bus (default channel `can0`, host ID `0xFD`)

## Project Layout

```
rebot_control/
├── config/
│   └── rebotarm_rs.yaml        # Config file: CAN / motors / temperature thresholds / return-to-zero
├── docs/
│   └── CLI_GUIDE.md            # Command-line interface (CLI) detailed guide
├── rebot/                      # Core package
│   ├── __init__.py             # Public API exports
│   ├── cli.py                  # Interactive REPL and per-joint CLI tool
│   ├── config.py               # Config layer: YAML loading, dataclasses and validation
│   └── controller.py           # Interface layer: ReBotRSMITController (pure control logic)
└── examples/                   # Examples and tutorial scripts
    ├── _bootstrap.py           # Path setup for direct script runs
    ├── mit_position_control.py # Full demo with editable targets and speeds
    ├── quick_start.py          # Tutorial 1: quick start
    ├── custom_config.py        # Tutorial 2: custom config file
    ├── monitor_status.py       # Tutorial 3: monitor while running
    ├── single_joint_adjust.py  # Tutorial 4: single-joint adjustments
    ├── read_joint_angles.py    # Tutorial 5: read actual positions
    ├── stop_options.py         # Tutorial 6: stop options
    └── recommended_structure.py # Tutorial 7: recommended program structure
```

Layer overview:

- **Config file** (`config/rebotarm_rs.yaml`): single source of truth for all tunable parameters — edit the YAML, never the code.
- **Config layer** (`rebot/config.py`): `load_config()` reads the YAML and builds validated dataclasses such as `ControllerConfig` (threshold ordering, frequencies, typo'd keys, etc.).
- **Interface layer** (`rebot/controller.py`): `ReBotRSMITController`, depends only on `ControllerConfig`; no hard-coded values or demo parameters.
- **Example layer** (`examples/`): runnable tutorial scripts and the full demo, matching the **Tutorial** section below.

## Features

- Continuous MIT position commands (200 Hz by default)
- Per-joint speed limits for 7 motors (J1–J6 arm + CAN ID 7 gripper; 20°/s default, smoothed in the control loop)
- Real-time MOS temperature monitoring of every motor (2 Hz by default)
- Three-level temperature protection:
  - ≥ 80°C: warning, keeps running
  - ≥ 100°C: stops motion, slowly returns to zero, then disables
  - ≥ 140°C: immediate emergency disable, no return-to-zero
- Safe exit: pressing Esc, Ctrl+C, or calling `arm.stop()` returns the arm to zero along a smoothstep trajectory before disabling
- Pressing Ctrl+C again during return-to-zero aborts it and disables immediately
- Reads actual positions before enabling to avoid sudden jumps at startup

## Installation

```bash
pip install motorbridge pynput pyyaml
```

`pynput` is used for Esc-key listening. Without it only the Esc shortcut is disabled; Ctrl+C and `stop()` still work.

## CAN Interface Setup

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

RobStride motors use 1 Mbps by default — adjust to your actual configuration.

## Usage

1. Edit `config/rebotarm_rs.yaml` to adjust the CAN interface, temperature thresholds, motor parameters, etc.
2. Edit `TARGET_ANGLES` and `JOINT_SPEEDS_DEG_S` at the top of `examples/mit_position_control.py`.
3. Run:

```bash
python3 examples/mit_position_control.py
```

### Config File (`config/rebotarm_rs.yaml`)

| Key | Description | Default |
|---|---|---|
| `can.channel` | CAN interface name | `can0` |
| `can.host_id` | Host ID | `0xFD` |
| `control.control_hz` | MIT command rate (not the motion speed) | 200 Hz |
| `control.telemetry_hz` | Temperature polling rate | 2 Hz |
| `temperatures.alarm_c` | Temperature warning threshold | 80°C |
| `temperatures.return_zero_c` | High-temperature return-to-zero threshold | 100°C |
| `temperatures.disconnect_c` | Emergency disable threshold | 140°C |
| `return_zero.max_speed_deg_s` | Peak speed of a normal return-to-zero | 30°/s |
| `return_zero.thermal_max_speed_deg_s` | Peak speed of a thermal return-to-zero | 30°/s |
| `return_zero.min_time_s` | Minimum return-to-zero duration | 3.0 s |
| `motors` | Per-motor ID / model / MIT `kp` / `kd` list | See YAML comments |

Keys omitted from the YAML fall back to the in-code defaults; unknown keys raise an error to catch typos.

## Command-Line Interface (rebot-cli)

`rebot_control` provides a built-in CLI tool `rebot-cli` for interactive joint tuning, manual teaching/drag monitoring, and scripted execution.

> For the comprehensive guide, see [**docs/CLI_GUIDE.md**](docs/CLI_GUIDE.md).

### Quick Examples

```bash
# 1. Start interactive REPL console
rebot-cli

# 2. Manual teaching / drag angle monitoring (Ctrl+C to stop and lock)
rebot-cli watch
rebot-cli watch --joint 1          # Watch only J1 with displacement delta

# 3. Read current angles and temperature without moving
rebot-cli status

# 4. One-shot movement commands
rebot-cli move --joint 1 --angle 25.0 --speed 15.0  # Move J1 to 25°
rebot-cli move --angles 10 15 20 -10 -20 30         # Move all 6 axes
rebot-cli zero                                      # Smooth return to zero
```

### Interactive Console Commands

Inside `rebot-cli`:
- `status` (or `st`): Display real-time 6-axis actual angles and motor temperatures
- `set <joint> <angle>`: Move a single joint to target angle in degrees (`set 1 25`)
- `step <joint> <delta>`: Relative adjustment in degrees (`step 2 5` or `step 2 -5`)
- `all <a1> <a2> <a3> <a4> <a5> <a6>`: Move all 6 joints simultaneously
- `watch [joint]` (or `drag`): Disable motors, enter manual teaching mode with high-frequency angle display, press `Ctrl+C` to exit
- `speed [joint] <deg_s>`: Dynamically adjust speed limit
- `zero` (or `z`): Smoothly return to 0° zero position
- `exit` (or `q`): Return to zero, disable motors, and exit cleanly

## Public API Reference

The `rebot` package exports the following public API from `rebot/__init__.py`:

| Name | Type | Description |
|---|---|---|
| `ReBotRSMITController` | class | MIT controller for the robot arm (main interface) |
| `load_config` | function | Load `ControllerConfig` from YAML |
| `ControllerConfig` | dataclass | Full controller configuration |
| `MotorConfig` | dataclass | Per-motor settings (ID, model, kp, kd) |
| `TemperatureThresholds` | dataclass | Three-level temperature thresholds |
| `ReturnZeroConfig` | dataclass | Return-to-zero trajectory parameters |
| `DEFAULT_CONFIG_PATH` | `Path` | Default config file path |
| `DEFAULT_MOTORS` | tuple | Default B601-RS motor list (includes gripper ID 7) |
| `GRIPPER_MOTOR_ID` | constant | Gripper CAN motor ID (7) |
| `GRIPPER_JOINT_ID` | constant | Gripper joint index in API (7, i.e. `set_joint_angle(7, …)`) |

### Controller Lifecycle

Typical call order (**call `start()` before sending motion commands**):

```
create instance → start() → set_max_speeds() → set_joint_angles() → … → stop()
```

| Stage | Method | Description |
|---|---|---|
| Create | `ReBotRSMITController(config=None)` | Loads `config/rebotarm_rs.yaml` when `config` is omitted |
| Connect | `connect()` | Open CAN, register motors, switch to MIT, enable; called automatically by `start()` |
| Start | `start(enable_esc=True, install_signal_handlers=True)` | Start control and temperature threads; optional Esc listener and SIGINT/SIGTERM handlers |
| Motion | `set_max_speeds()` / `set_joint_angles()` / `set_joint_angle()` | Set speeds and target angles (degrees) |
| Query | `get_target_angles()` / `get_command_angles()` / `read_joint_angles()` | Read targets, smoothed commands, actual positions (degrees) |
| Stop | `stop()` or `request_stop()` | Default: slow return-to-zero, then disable and close CAN |

### `ReBotRSMITController` API

#### Construction and Connection

| Method | Parameters | Returns | Description |
|---|---|---|---|
| `__init__(config=None)` | `config`: `ControllerConfig \| None` | — | Create controller; loads default YAML when `config` is `None` |
| `connect()` | — | `None` | Connect CAN, MIT mode, enable, then read current angles; safe to call repeatedly |
| `disable_motors()` | — | `None` | Disable all motors without closing CAN (for passive position reading) |
| `start(...)` | `enable_esc`: Esc listener; `install_signal_handlers`: Ctrl+C / SIGTERM | `None` | Start background threads; safe to call repeatedly |

#### Motion Control (units: degrees, deg/s)

| Method | Parameters | Returns | Description |
|---|---|---|---|
| `set_joint_angles(angles_deg)` | Sequence of 7 values, J1–J6 + gripper J7 | `None` | Set all joint and gripper targets |
| `set_joint_angle(joint_id, angle_deg)` | `joint_id`: 1–7; J7 is gripper | `None` | Change a single joint/gripper target |
| `set_max_speeds(speeds_deg_s)` | Sequence of 7 max speeds | `None` | Per-joint/gripper speed ceiling for smoothing |

#### State Queries

| Method / Attribute | Returns | Description |
|---|---|---|
| `get_target_angles()` | `list[float]` | User-set final target angles (degrees) |
| `get_command_angles()` | `list[float]` | Smoothed angles actually being sent (degrees) |
| `read_joint_angles()` | `list[float]` | Synchronous read of actual mechanical positions (degrees); uses CAN |
| `last_temperatures` | `list[float \| None]` | Latest MOS temperatures (°C) per motor; `None` if unreadable |
| `last_error` | `Exception \| None` | Error recorded when the control thread hits a communication failure |
| `is_stopped` | `bool` | Whether the safe shutdown sequence has finished |
| `config` | `ControllerConfig` | Active configuration object |

#### Stop and Safe Exit

| Method | Parameters | Description |
|---|---|---|
| `stop(return_to_zero=True, wait=True)` | `return_to_zero`: return to zero first; `wait`: block until done | Most common stop interface |
| `request_stop(...)` | `reason`, `return_to_zero`, `thermal`, `emergency`, `wait` | Advanced stop; `emergency=True` disables immediately without return-to-zero |
| `wait_until_stopped(timeout=None)` | `timeout`: max wait in seconds | Block until shutdown completes; returns whether it finished before timeout |

### Configuration API

```python
from rebot import load_config, ControllerConfig, DEFAULT_CONFIG_PATH

config = load_config()                        # default file
config = load_config("config/rebotarm_rs.yaml")

print(DEFAULT_CONFIG_PATH)
```

`ControllerConfig.from_yaml(path)` works the same as `load_config(path)`.

## Tutorial

All examples can be run from the project root. Source code lives in `examples/`.

Each example documents **Expected motion** at the top of the file and prints an `[Expected / 预期]` line at startup so you can compare with what the arm actually does.

| Tutorial | Example file | Run command |
|---|---|---|
| 1. Quick start | `examples/quick_start.py` | `python3 examples/quick_start.py` |
| 2. Custom config | `examples/custom_config.py` | `python3 examples/custom_config.py` |
| 3. Monitor status | `examples/monitor_status.py` | `python3 examples/monitor_status.py` |
| 4. Single joint | `examples/single_joint_adjust.py` | `python3 examples/single_joint_adjust.py` |
| 5. Read positions | `examples/read_joint_angles.py` | `python3 examples/read_joint_angles.py` |
| 6. Stop options | `examples/stop_options.py` | `python3 examples/stop_options.py default` |
| 7. Program structure | `examples/recommended_structure.py` | `python3 examples/recommended_structure.py` |
| Full editable demo | `examples/mit_position_control.py` | `python3 examples/mit_position_control.py` |

### 1. Quick Start (minimal example)

See `examples/quick_start.py`: create controller → `start()` → set speeds and targets → **wait for motion** → `stop()`.

> `set_joint_angles()` only updates targets; the arm ramps at `set_max_speeds()`. Calling `stop()` immediately may show almost no motion toward the target.

### 2. Custom Config File

See `examples/custom_config.py`: pass `load_config("config/rebotarm_rs.yaml")` to `ReBotRSMITController`.

Pass another YAML on the command line:

```bash
python3 examples/custom_config.py config/rebotarm_rs.yaml
```

### 3. Monitor While Running

See `examples/monitor_status.py`. For an editable full demo, see `examples/mit_position_control.py`.

Notes:

- **Target vs sent**: `set_joint_angles()` sets the target; the control loop ramps toward it according to `set_max_speeds()`, so sent angles lag behind targets.
- **Temperature**: refreshed by the background thread at `telemetry_hz`; read `last_temperatures` instead of polling CAN yourself.

### 4. Single-Joint Adjustments

See `examples/single_joint_adjust.py`. Default targets: J1=+25°, J2=+15°, J3=+15°, J4=-15°, J5/J6=0°, gripper J7=180°. `joint_id` 1–6 are arm joints; **7 is the gripper** (CAN ID 7).

### 5. Read Actual Positions

See `examples/read_joint_angles.py`. Flow: `connect()` (MIT mode, enable, read current angles) → `disable_motors()` (disable so you can move the arm by hand) → loop `read_joint_angles()` to print actual angles. Press Ctrl+C to exit; `stop(return_to_zero=False)` closes CAN without return-to-zero.

`read_joint_angles()` uses CAN synchronously — avoid calling it at very high rates; use `get_command_angles()` to monitor commanded motion.

### 6. Stop Options

See `examples/stop_options.py` — pick a mode via CLI argument:

```bash
python3 examples/stop_options.py default      # slow return-to-zero (recommended)
python3 examples/stop_options.py no_return    # disable without return-to-zero
python3 examples/stop_options.py async        # stop(wait=False) + wait_until_stopped()
python3 examples/stop_options.py emergency    # emergency disable, no return-to-zero
```

Interactive exit: **Esc** or **Ctrl+C** during operation triggers a return-to-zero similar to `stop()`; a second **Ctrl+C** during return-to-zero aborts it and disables immediately.

### 7. Recommended Program Structure

See `examples/recommended_structure.py`: `try` / `except` / `finally` template that always calls `stop()` safely.

## Units

- External interfaces (target angles, speeds): degrees, deg/s
- MotorBridge MIT commands internally: radians, rad/s

## Notes

- **No software limits**: this repository does not provide joint or workspace software limits; see **Warning** above before use.
- Make sure the area around the arm is clear before running. For the first run, use small angles and low speeds, and stay within roughly 70% of the workspace.
- On a communication error the program does not attempt to return to zero — it disables immediately.
- `control_hz` is only the command rate; the actual motion speed is set by `set_max_speeds`.
