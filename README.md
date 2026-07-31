# mujoco_micro ROS2 workspace

This is a conventional ROS2 Humble colcon workspace.

```text
mujoco_micro/
└── src/
    └── mujoco_micro/
        ├── package.xml
        ├── CMakeLists.txt
        ├── launch/
        ├── config/
        ├── include/
        ├── src/
        ├── scripts/
        └── tools/
```

Build it on Ubuntu 22.04 after sourcing the ROS2 workspace that contains `custom_msgs`:

```bash
sudo apt install libopencv-dev
```

```bash
source /opt/ros/humble/setup.bash
source ~/foot_ws/install/setup.bash
cd ~/mujoco_micro
colcon build --symlink-install
source install/setup.bash
ros2 launch mujoco_micro mujoco_micro.launch.py dry_run:=true
```
