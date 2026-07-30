#!/usr/bin/env bash
set -euo pipefail

read_topics=(
  /ecat/sn1966149/app1/read
  /ecat/sn2228252/app1/read
  /ecat/sn2228252/app2/read
  /ecat/sn2228252/app3/read
  /ecat/sn2228252/app4/read
  /ecat/sn2228252/app5/read
  /ecat/sn2228252/app6/read
  /ecat/sn2228252/app7/read
)
write_topics=(
  /ecat/sn2228252/app2/write
  /ecat/sn2228252/app3/write
  /ecat/sn2228252/app4/write
  /ecat/sn2228252/app5/write
  /ecat/sn2228252/app6/write
  /ecat/sn2228252/app7/write
)

available="$(ros2 topic list)"
missing=0

echo '===== Required feedback topics ====='
for topic in "${read_topics[@]}"; do
  if grep -Fxq "$topic" <<<"$available"; then
    printf '[OK]      %-38s %s\n' "$topic" "$(ros2 topic type "$topic" 2>/dev/null || true)"
  else
    printf '[MISSING] %s\n' "$topic"
    missing=1
  fi
done

echo
echo '===== Command topics (appear after driver/controller publishers start) ====='
for topic in "${write_topics[@]}"; do
  if grep -Fxq "$topic" <<<"$available"; then
    printf '[OK]      %-38s %s\n' "$topic" "$(ros2 topic type "$topic" 2>/dev/null || true)"
  else
    printf '[INFO]    not visible yet: %s\n' "$topic"
  fi
done

echo
echo 'Expected types:'
echo '  IMU:        sensor_msgs/msg/Imu'
echo '  RC:         custom_msgs/msg/ReadDJIRC'
echo '  Motor read: custom_msgs/msg/ReadDmMotor'
echo '  Motor cmd:  custom_msgs/msg/WriteDmMotorMITControl'

exit "$missing"
