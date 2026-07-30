# Roll 控制增量说明

本工作空间以 `ssybh2/Micro_to_Macro` 当前 `main` 源码为基础，只增加 Roll 通道；原有功能保持：

- `right_y`：前进/后退目标速度
- `left_x`：Yaw 目标角速度
- `right_x`：新增 Roll 目标角
- Pitch 平衡、前后位置/速度、Yaw 差动扭矩、VMC、MIT 关节位置保持、遥控开关状态机均保留

## Roll 执行逻辑

Roll 不使用轮毂差动扭矩，而是改变左右腿轮心目标 Y：

```text
right_x -> 目标 Roll 角 -> Roll PD -> 左右腿总高度差

left_target_y  = base_target_y + 0.5 * leg_difference
right_target_y = base_target_y - 0.5 * leg_difference
```

IK 与 VMC 使用同一对左右目标高度，避免位置项和 VMC 相互对抗。Roll 只在轮毂平衡已经进入 `BALANCE` 后生效；`LEG_POSITIONING` 阶段仍使用相同的基础腿高。

## 安全默认

`config/mujoco_micro.yaml` 中：

```yaml
roll:
  enable: false
```

这是为了让替换后的第一次运行完全保持原有行为。悬架确认方向后改成：

```yaml
roll:
  enable: true
```

## 最常用参数

```yaml
imu:
  roll_axis: "roll"
  roll_rate_axis: "x"
  roll_sign: 1.0
  roll_rate_sign: 1.0

roll:
  enable: true
  trim_deg: 0.0

  kp_leg_difference_m_per_rad: 0.080
  kd_leg_difference_m_per_rad_s: 0.008

  max_leg_difference_m: 0.016
  leg_difference_slew_rate_mps: 0.040

  max_target_roll_deg: 5.0
  target_slew_rate_deg_s: 20.0
  rc_deadband: 0.08

  rc_sign: 1.0
  output_sign: 1.0
```

### 三个方向参数不要混淆

- `imu.roll_sign`：IMU 测得的 Roll 正负方向。
- `roll.rc_sign`：右摇杆横向命令的正负方向。
- `roll.output_sign`：Roll 控制器左右腿伸缩方向。自动修正越修越歪时只翻转它。

### 最大两腿差

`max_leg_difference_m` 表示：

```text
abs(left_target_y - right_target_y)
```

例如 `0.016` 表示左右腿目标总差最大 16 mm，即平均腿高不变时，单腿最多相对基础目标偏移 8 mm。

## 新增调试数据

原 `/vmc/debug` 的既有索引 0~50 完全不变，新增值追加在末尾：

| 索引 | 数据 |
|---:|---|
| 51 | filtered roll [rad] |
| 52 | filtered roll rate [rad/s] |
| 53 | target roll（trim + RC）[rad] |
| 54 | RC roll target command [rad] |
| 55 | roll error [rad] |
| 56 | total leg difference [m] |
| 57 | left target y [m] |
| 58 | right target y [m] |
| 59 | raw RC right_x |

监视命令：

```bash
ros2 run mujoco_micro mujoco_micro_debug_monitor.py
```

## 首次方向测试

1. 可靠悬架机器人。
2. 保持 `roll.enable: false`，先确认 Pitch、前后和 Yaw 与原程序一致。
3. 将 `roll.enable` 改为 `true`，先设置较小限制：

```yaml
roll:
  max_leg_difference_m: 0.008
  max_target_roll_deg: 3.0
  kp_leg_difference_m_per_rad: 0.040
  kd_leg_difference_m_per_rad_s: 0.004
```

4. 轻微手动倾斜车身。若左右腿动作让 Roll 更严重，修改：

```yaml
roll:
  output_sign: -1.0
```

5. 推动 `right_x`。若遥控方向与预期相反，只修改：

```yaml
roll:
  rc_sign: -1.0
```

6. 若日志中的 Roll 正负与物理倾斜方向相反，修改：

```yaml
imu:
  roll_sign: -1.0
  roll_rate_sign: -1.0
```

## 编译

```bash
cd ~/mujoco_micro
rm -rf build install log
source /opt/ros/humble/setup.bash
source ~/foot_ws/install/setup.bash
colcon build --symlink-install --packages-select mujoco_micro
source install/setup.bash
```

第一次只计算不输出：

```bash
ros2 launch mujoco_micro mujoco_micro.launch.py dry_run:=true
```

实机：

```bash
ros2 launch mujoco_micro mujoco_micro.launch.py dry_run:=false
```
