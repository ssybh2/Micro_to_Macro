# mujoco_micro

Standard ROS2 Humble `ament_cmake` package migrated from the controller validated in MuJoCo.

## Package layout

```text
mujoco_micro/
├── CMakeLists.txt
├── package.xml
├── launch/mujoco_micro.launch.py
├── config/mujoco_micro.yaml
├── include/mujoco_micro/
├── src/
│   ├── mujoco_micro_node.cpp
│   ├── control_core.cpp
│   └── kinematics.cpp
├── scripts/
└── tools/
```

## Build

The workspace must be overlaid on the workspace that provides `custom_msgs`.

```bash
source /opt/ros/humble/setup.bash
source ~/foot_ws/install/setup.bash
cd ~/mujoco_micro
colcon build --symlink-install --packages-select mujoco_micro
source install/setup.bash
```

## Start safely

```bash
ros2 launch mujoco_micro mujoco_micro.launch.py dry_run:=true
```

After confirming topics, IMU orientation, encoder signs, zero positions and `/vmc/debug`:

```bash
ros2 launch mujoco_micro mujoco_micro.launch.py dry_run:=false
```

Enable RC velocity input only after stationary balancing works:

```bash
ros2 launch mujoco_micro mujoco_micro.launch.py \
  dry_run:=false velocity_command_enable:=true
```

## Main files

- Node: `src/mujoco_micro_node.cpp`
- Launch: `launch/mujoco_micro.launch.py`
- Parameters: `config/mujoco_micro.yaml`
- Debug topic: `/vmc/debug`

## Roll extension

The workspace-level `ROLL_CONTROL_README_CN.md` documents the additive right_x Roll controller. Existing `/vmc/debug` indices 0-50 are preserved; Roll values are appended at indices 51-59.

## Height control and ONNX residual policy

The installed `models/policy.onnx` is evaluated in the C++ control loop through
OpenCV DNN. The controller preserves the existing cascade controller as the
baseline and adds the policy output as a common physical wheel-torque residual
before yaw differential torque is applied.

- `left_y`: absolute height command, `-1/0/+1` maps to `0.090/0.120/0.160 m`.
- Height target slew rate: `0.060 m/s`.
- Policy action: clamped to `[-1, 1]`, then multiplied by `0.060 Nm`.
- Disable only the residual policy with `policy_enable:=false`.

First run with motor output disabled:

```bash
ros2 launch mujoco_micro mujoco_micro.launch.py dry_run:=true policy_enable:=true
ros2 run mujoco_micro mujoco_micro_debug_monitor.py
```

Debug indices 60-74 contain height/policy raw values. Indices 75-85 contain
the exact normalized observation vector sent to the model.

| Index | Value |
|---:|---|
| 60 | raw RC `left_y` |
| 61 / 62 / 63 | height command / limited target / target rate |
| 64 / 65 | measured mean leg height / measured height rate |
| 66 / 67 | policy wheel-only position / velocity (no pitch compensation) |
| 68 / 69 | previous action input / current clipped action |
| 70 / 71 | requested / actually applied residual torque per wheel |
| 72 | combined common physical torque per wheel |
| 73 / 74 | inference time in microseconds / policy active flag |
| 75-85 | normalized policy observation indices 0-10 |

## Gravity-referenced attitude

With `imu.reference_mode: gravity` (the default), Pitch and Roll are measured
against the world gravity/up direction extracted from the fused IMU quaternion.
The pose at RC switch 1 is no longer used as the attitude zero. Switch 3 resets
the wheel origin and arms the controller once the gravity reference is valid.

- `imu.gravity_source: orientation` is recommended and rejects translational
  acceleration by relying on the IMU's fused quaternion.
- `imu.gravity_source: accelerometer` is a fallback for IMUs without a valid
  fused orientation and requires stationary acceleration magnitude near gravity.
- `imu.mount_roll_deg` and `imu.mount_pitch_deg` compensate fixed IMU mounting
  misalignment and persist across power cycles.

Debug indices 86-92 are gravity up-vector X/Y/Z in the IMU frame, raw
acceleration norm, reference-ready flag, gravity-mode flag, and orientation-source flag.

## Recovery policy and automatic handoff

With `recovery.enable: true`, RC switch 3 no longer requires the robot to
already be inside the NORMAL 10-degree arm window. It runs the packaged
`models/recovery_policy.onnx` first and then changes to the existing cascade
controller plus `policy.onnx`:

```text
DISARMED -> RECOVERY -> NORMAL
 switch 3    stable       cascade + residual policy
 switch 2 from either active state immediately disables all motors
```

RECOVERY evaluates its 24-to-3 actor at 100 Hz. The actions are common wheel
torque, virtual-leg height rate, and virtual-leg fore/aft angle. Hip torques
use the training-side explicit `Kp=7`, `Kd=0.28`, `+/-1.5 Nm` PD law. Extension
stays locked until COM/wheel/gravity alignment is within 5 degrees for 0.10 s,
and relocks outside 8 degrees. The automatic handoff requires pitch,
alignment, pitch rate, 120 mm height, and wheel speed to remain within their
configured success bounds for 0.15 s. Wheel torque and hip targets blend into
NORMAL over `recovery.handoff_blend_s`.

Use `recovery_enable:=false` to restore direct NORMAL arming. Recovery safety,
gate, model and handoff parameters are under `recovery:` in the YAML. Debug
indices 93-115 contain the recovery state/action values, and indices 116-139
are the exact normalized observation vector sent to the recovery model.

Always validate the first run with motor output disabled:

```bash
ros2 launch mujoco_micro mujoco_micro.launch.py dry_run:=true recovery_enable:=true
ros2 run mujoco_micro mujoco_micro_debug_monitor.py
```
