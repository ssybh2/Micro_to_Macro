<div align="center">

# Reinforcement Learning Wheel-Legged Robot

**Micro_to_Macro — Sim-to-Real control for a self-balancing wheel-legged robot**

<p>
  <img src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 22.04">
  <img src="https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white" alt="C++17">
  <img src="https://img.shields.io/badge/ONNX-Residual%20RL-005CED?logo=onnx&logoColor=white" alt="ONNX Residual RL">
  <img src="https://img.shields.io/badge/Control-VMC%20%2B%20Cascade-2F855A" alt="VMC and Cascade Control">
</p>

<img src="docs/assets/D295D60F-BB3B-4AB2-A5D5-64C60618934B.png" width="520" alt="Wheel-legged self-balancing robot">

</div>

## Overview

`Micro_to_Macro` is a ROS 2 Humble control workspace for a **wheel-legged self-balancing robot**, developed around a hybrid control stack that combines model-based control with reinforcement learning.

The controller integrates **five-bar kinematics, VMC, wheel-balance control, yaw/roll/height control, an ONNX residual policy, and an ONNX recovery policy** for deployment on real hardware through ROS 2 and EtherCAT.

## Demo

<div align="center">
  <a href="docs/assets/copy_81C3F3D5-9555-45DA-98A7-C2422D7B46FF.mov">
    <img src="docs/assets/D295D60F-BB3B-4AB2-A5D5-64C60618934B.png" width="760" alt="Robot demo video">
  </a>
  <br>
  <sub>▶ Click the image to play the real-robot demo video</sub>
</div>


## Quick Start

### 1. Dependencies

```bash
sudo apt install libopencv-dev
source /opt/ros/humble/setup.bash
source ~/foot_ws/install/setup.bash
```

The overlaid `foot_ws` workspace must provide the EtherCAT interface and `custom_msgs`.

### 2. Build

```bash
git clone https://github.com/ssybh2/Micro_to_Macro.git ~/mujoco_micro
cd ~/mujoco_micro
colcon build --symlink-install --packages-select mujoco_micro
source install/setup.bash
```

### 3. First run — motor output disabled

```bash
ros2 launch mujoco_micro mujoco_micro.launch.py dry_run:=true
```

Monitor the controller state with:

```bash
ros2 run mujoco_micro mujoco_micro_debug_monitor.py
```


## Documentation

- Detailed package documentation: [`src/mujoco_micro/README.md`](src/mujoco_micro/README.md)
- Roll controller notes: [`ROLL_CONTROL_README_CN.md`](ROLL_CONTROL_README_CN.md)
- Main parameters: [`src/mujoco_micro/config/mujoco_micro.yaml`](src/mujoco_micro/config/mujoco_micro.yaml)

---

<div align="center">
  <sub>Robotics · Reinforcement Learning · Sim-to-Real · Wheel-Legged Control</sub>
</div>
