#!/usr/bin/env bash
set -euo pipefail

messages=(
  custom_msgs/msg/ReadDJIRC
  custom_msgs/msg/ReadDmMotor
  custom_msgs/msg/WriteDmMotorMITControl
)

for msg in "${messages[@]}"; do
  echo "===== ${msg} ====="
  ros2 interface show "${msg}"
  echo
done

cat <<'EOF'
mujoco_micro_node 需要的关键字段：
ReadDJIRC: online, right_switch, right_y, left_x
ReadDmMotor: online, disabled, enabled, overvoltage, undervoltage, overcurrent,
             mos_overtemperature, rotor_overtemperature, communication_lost,
             overload, position, velocity, torque
WriteDmMotorMITControl: enable, p_des, v_des, kp, kd, torque
EOF
