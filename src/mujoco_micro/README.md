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
