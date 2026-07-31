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

- `left_y`: absolute height command, `-1/0/+1` maps to `0.090/0.125/0.160 m`.
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
