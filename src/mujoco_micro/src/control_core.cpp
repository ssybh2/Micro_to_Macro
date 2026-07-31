#include "mujoco_micro/control_core.hpp"

#include <stdexcept>

namespace mujoco_micro
{

bool normalize_quaternion(Quaternion & q)
{
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!std::isfinite(norm) || norm < 1.0e-12) {
    return false;
  }
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  return true;
}

Quaternion relative_quaternion(const Quaternion & reference, Quaternion current)
{
  Quaternion ref = reference;
  normalize_quaternion(ref);
  normalize_quaternion(current);
  const double dot = ref.w * current.w + ref.x * current.x + ref.y * current.y + ref.z * current.z;
  if (dot < 0.0) {
    current.w = -current.w;
    current.x = -current.x;
    current.y = -current.y;
    current.z = -current.z;
  }
  // conjugate(reference) * current
  Quaternion result;
  result.w = ref.w * current.w + ref.x * current.x + ref.y * current.y + ref.z * current.z;
  result.x = ref.w * current.x - ref.x * current.w - ref.y * current.z + ref.z * current.y;
  result.y = ref.w * current.y + ref.x * current.z - ref.y * current.w - ref.z * current.x;
  result.z = ref.w * current.z - ref.x * current.y + ref.y * current.x - ref.z * current.w;
  normalize_quaternion(result);
  return result;
}

double quaternion_roll(const Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.x + q.y * q.z),
    1.0 - 2.0 * (q.x * q.x + q.y * q.y));
}

double quaternion_pitch(const Quaternion & q)
{
  const double value = clamp_value(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0);
  return std::asin(value);
}

bool normalize_vector(Vector3 & vector)
{
  const double norm = std::hypot(vector.x, std::hypot(vector.y, vector.z));
  if (!std::isfinite(norm) || norm < 1.0e-12) {
    return false;
  }
  vector.x /= norm;
  vector.y /= norm;
  vector.z /= norm;
  return true;
}

Vector3 world_up_axis_in_body(const Quaternion & orientation)
{
  Quaternion q = orientation;
  if (!normalize_quaternion(q)) {
    return {};
  }
  // ROS orientation rotates vectors from the IMU/body frame into the world
  // frame. R(q)^T * world_Z therefore expresses the gravity-opposing world
  // up axis in IMU coordinates and is independent of yaw.
  Vector3 up{
    2.0 * (q.x * q.z - q.w * q.y),
    2.0 * (q.y * q.z + q.w * q.x),
    1.0 - 2.0 * (q.x * q.x + q.y * q.y)};
  (void)normalize_vector(up);
  return up;
}

double gravity_roll(const Vector3 & up_axis_in_body)
{
  return std::atan2(up_axis_in_body.y, up_axis_in_body.z);
}

double gravity_pitch(const Vector3 & up_axis_in_body)
{
  return std::atan2(
    -up_axis_in_body.x,
    std::hypot(up_axis_in_body.y, up_axis_in_body.z));
}

double JointCalibration::motor_to_joint(const double motor_position) const
{
  if (!std::isfinite(motor_position) || std::abs(motor_sign) < 1.0e-12 ||
    std::abs(ratio) < 1.0e-12)
  {
    return 0.0;
  }
  return (motor_position - motor_zero_rad) * motor_sign / ratio + joint_zero_rad;
}

double JointCalibration::joint_to_motor(const double joint_position) const
{
  if (!std::isfinite(joint_position) || std::abs(motor_sign) < 1.0e-12 ||
    std::abs(ratio) < 1.0e-12)
  {
    return 0.0;
  }
  return motor_zero_rad + (joint_position - joint_zero_rad) * ratio / motor_sign;
}

VmcOutput calculate_vmc(
  const FiveBarState & state, const double pitch, const double supported_mass,
  const VmcConfig & config)
{
  VmcOutput out;
  if (!state.valid || !std::isfinite(pitch) || !std::isfinite(supported_mass)) {
    return out;
  }

  out.fx = config.kx_n_per_m * (config.target_x_m - state.x) -
    config.dx_ns_per_m * state.x_dot;
  out.fy = config.ky_n_per_m * (config.target_y_m - state.y) -
    config.dy_ns_per_m * state.y_dot;

  if (config.gravity_compensation) {
    out.fy += supported_mass * 9.81 * std::cos(pitch);
    if (config.horizontal_pitch_gravity_compensation) {
      out.fx -= supported_mass * 9.81 * std::sin(pitch);
    }
  }

  out.tau_a = state.j11 * out.fx + state.j21 * out.fy;
  out.tau_b = state.j12 * out.fx + state.j22 * out.fy;
  const double limit = std::abs(config.joint_torque_limit_nm);
  out.tau_a = clamp_value(out.tau_a, -limit, limit);
  out.tau_b = clamp_value(out.tau_b, -limit, limit);
  return out;
}

void BalanceController::configure(const BalanceConfig & config)
{
  config_ = config;
  if (config_.mode != "cascade" && config_.mode != "lqr") {
    throw std::runtime_error("balance.mode must be cascade or lqr");
  }
  if (!(config_.control_period_s > 0.0) || !(config_.wheel_radius_m > 0.0)) {
    throw std::runtime_error("control period and wheel radius must be positive");
  }
  if (!(config_.torque_limit_each_nm > 0.0) ||
    config_.torque_limit_each_nm > config_.hard_torque_limit_each_nm)
  {
    throw std::runtime_error("wheel torque limit must be positive and <= hard limit");
  }
  if (config_.velocity_blend < 0.0 || config_.velocity_blend > 1.0) {
    throw std::runtime_error("velocity_blend must be within [0, 1]");
  }
  const auto valid_sign = [](const double value) {
      return std::isfinite(value) && std::abs(std::abs(value) - 1.0) < 1.0e-9;
    };
  if (!valid_sign(config_.left_encoder_sign) || !valid_sign(config_.right_encoder_sign) ||
    !valid_sign(config_.pitch_position_compensation_sign) ||
    !valid_sign(config_.pitch_rate_compensation_sign) ||
    !valid_sign(config_.output_gain_sign) || !valid_sign(config_.left_motor_sign) ||
    !valid_sign(config_.right_motor_sign) || !valid_sign(config_.rc_forward_sign) ||
    !valid_sign(config_.rc_yaw_sign) || !valid_sign(config_.yaw_output_sign) ||
    !valid_sign(config_.cascade_position_to_pitch_sign))
  {
    throw std::runtime_error("all balance direction signs must be exactly +1 or -1");
  }
  if (config_.cascade_attitude_k_pitch < 0.0 ||
    config_.cascade_attitude_k_pitch_rate < 0.0 ||
    config_.cascade_position_kp_rad_per_m < 0.0 ||
    config_.cascade_velocity_kd_rad_per_mps < 0.0 ||
    config_.cascade_position_ki_rad_per_m_s < 0.0)
  {
    throw std::runtime_error("cascade gains must be non-negative");
  }

  left_unwrapper_.configure(config_.motor_position_wrap_half_range);
  right_unwrapper_.configure(config_.motor_position_wrap_half_range);
  pitch_filter_.configure(config_.pitch_filter_hz, config_.control_period_s);
  pitch_rate_filter_.configure(config_.pitch_rate_filter_hz, config_.control_period_s);
  wheel_velocity_filter_.configure(config_.wheel_velocity_filter_hz, config_.control_period_s);
  position_velocity_filter_.configure(config_.position_velocity_filter_hz, config_.control_period_s);
  outer_velocity_filter_.configure(config_.outer_velocity_filter_hz, config_.control_period_s);
  yaw_rate_filter_.configure(config_.yaw_rate_filter_hz, config_.control_period_s);
  yaw_accel_filter_.configure(config_.yaw_accel_filter_hz, config_.control_period_s);
  reset();
}

void BalanceController::reset()
{
  arm_initialized_ = false;
  fd_initialized_ = false;
  last_left_sequence_ = 0;
  last_right_sequence_ = 0;
  last_position_m_ = 0.0;
  velocity_from_position_mps_ = 0.0;
  fd_elapsed_s_ = 0.0;
  observer_mean_wheel_angle_rad_ = 0.0;
  policy_wheel_origin_angle_rad_ = 0.0;
  policy_wheel_origin_initialized_ = false;
  target_position_m_ = 0.0;
  target_velocity_mps_ = 0.0;
  target_yaw_rate_rad_s_ = 0.0;
  cascade_pitch_setpoint_rad_ = 0.0;
  cascade_position_integral_m_s_ = 0.0;
  auto_trim_stationary_s_ = 0.0;
  yaw_rate_initialized_ = false;
  yaw_diff_command_nm_ = 0.0;
}

void BalanceController::calibrate_wheels(
  const double left_raw_position, const double right_raw_position)
{
  left_unwrapper_.reset(left_raw_position);
  right_unwrapper_.reset(right_raw_position);
  reset();
}

void BalanceController::update_observer(const BalanceInput & input, BalanceDebug & debug)
{
  const double dt = clamp_value(input.dt, 1.0e-6, 0.050);
  const double pitch = pitch_filter_.update(input.pitch_rad, dt);
  const double pitch_rate = pitch_rate_filter_.update(input.pitch_rate_rad_s, dt);
  const double left_angle = config_.left_encoder_sign *
    left_unwrapper_.update(input.left_wheel_position_rad);
  const double right_angle = config_.right_encoder_sign *
    right_unwrapper_.update(input.right_wheel_position_rad);
  const double mean_angle = 0.5 * (left_angle + right_angle);
  const double mean_rate_raw = 0.5 * (
    config_.left_encoder_sign * input.left_wheel_velocity_rad_s +
    config_.right_encoder_sign * input.right_wheel_velocity_rad_s);
  const double mean_rate = wheel_velocity_filter_.update(mean_rate_raw, dt);

  observer_mean_wheel_angle_rad_ = mean_angle;
  const double policy_origin = policy_wheel_origin_initialized_ ?
    policy_wheel_origin_angle_rad_ : mean_angle;
  debug.policy_wheel_position_m = config_.wheel_radius_m * (mean_angle - policy_origin);
  debug.policy_wheel_velocity_mps = config_.wheel_radius_m * mean_rate_raw;

  debug.position_m = config_.wheel_radius_m * (
    mean_angle + config_.pitch_position_compensation_sign * pitch);
  debug.velocity_motor_based_mps = config_.wheel_radius_m * (
    mean_rate + config_.pitch_rate_compensation_sign * pitch_rate);

  fd_elapsed_s_ += dt;
  const bool new_pair = input.left_sequence != last_left_sequence_ &&
    input.right_sequence != last_right_sequence_;
  if (new_pair) {
    if (fd_initialized_) {
      const double encoder_dt = clamp_value(fd_elapsed_s_, 1.0e-6, 0.10);
      const double raw_fd = (debug.position_m - last_position_m_) / encoder_dt;
      velocity_from_position_mps_ = position_velocity_filter_.update(raw_fd, encoder_dt);
    } else {
      position_velocity_filter_.reset(debug.velocity_motor_based_mps);
      velocity_from_position_mps_ = debug.velocity_motor_based_mps;
      fd_initialized_ = true;
    }
    last_position_m_ = debug.position_m;
    fd_elapsed_s_ = 0.0;
    last_left_sequence_ = input.left_sequence;
    last_right_sequence_ = input.right_sequence;
  }

  debug.velocity_position_fd_mps = velocity_from_position_mps_;
  debug.velocity_mps = config_.velocity_blend * debug.velocity_motor_based_mps +
    (1.0 - config_.velocity_blend) * velocity_from_position_mps_;
  debug.pitch_filtered_rad = pitch;
  debug.pitch_rate_filtered_rad_s = pitch_rate;
}

void BalanceController::arm(const BalanceInput & input)
{
  pitch_filter_.reset(input.pitch_rad);
  pitch_rate_filter_.reset(input.pitch_rate_rad_s);
  const double mean_rate_raw = 0.5 * (
    config_.left_encoder_sign * input.left_wheel_velocity_rad_s +
    config_.right_encoder_sign * input.right_wheel_velocity_rad_s);
  wheel_velocity_filter_.reset(mean_rate_raw);
  fd_initialized_ = false;
  fd_elapsed_s_ = 0.0;

  BalanceDebug debug;
  update_observer(input, debug);
  policy_wheel_origin_angle_rad_ = observer_mean_wheel_angle_rad_;
  policy_wheel_origin_initialized_ = true;
  target_position_m_ = debug.position_m;
  target_velocity_mps_ = 0.0;
  target_yaw_rate_rad_s_ = 0.0;
  cascade_position_integral_m_s_ = 0.0;
  outer_velocity_filter_.reset(0.0);
  const double total_trim = config_.manual_trim_rad + auto_trim_rad_;
  cascade_pitch_setpoint_rad_ = config_.cascade_arm_bumpless ?
    clamp_value(
      debug.pitch_filtered_rad - total_trim,
      -config_.cascade_pitch_limit_rad, config_.cascade_pitch_limit_rad) : 0.0;
  yaw_rate_filter_.reset(input.yaw_rate_rad_s);
  yaw_accel_filter_.reset(0.0);
  last_yaw_rate_filtered_rad_s_ = input.yaw_rate_rad_s;
  yaw_rate_initialized_ = true;
  yaw_diff_command_nm_ = 0.0;
  arm_initialized_ = true;
}

double BalanceController::calculate_cascade(
  const double pitch, const double pitch_rate, const double position_error,
  const double velocity_error, const double dt, BalanceDebug & debug)
{
  const double outer_velocity = outer_velocity_filter_.update(velocity_error, dt);
  const double ex = clamp_value(
    position_error, -config_.cascade_position_error_limit_m,
    config_.cascade_position_error_limit_m);
  const double ev = clamp_value(
    outer_velocity, -config_.cascade_velocity_error_limit_mps,
    config_.cascade_velocity_error_limit_mps);

  if (config_.cascade_integral_enable) {
    cascade_position_integral_m_s_ = clamp_value(
      cascade_position_integral_m_s_ + ex * dt,
      -config_.cascade_integral_limit_m_s, config_.cascade_integral_limit_m_s);
  } else {
    cascade_position_integral_m_s_ = 0.0;
  }

  const double outer_command = config_.cascade_position_kp_rad_per_m * ex +
    config_.cascade_velocity_kd_rad_per_mps * ev +
    config_.cascade_position_ki_rad_per_m_s * cascade_position_integral_m_s_;
  const double raw_setpoint = config_.cascade_position_to_pitch_sign * outer_command;
  const double limited_setpoint = clamp_value(
    raw_setpoint, -config_.cascade_pitch_limit_rad, config_.cascade_pitch_limit_rad);
  debug.outer_saturated = std::abs(raw_setpoint - limited_setpoint) > 1.0e-12;

  const double max_step = config_.cascade_pitch_slew_rate_rad_s * dt;
  cascade_pitch_setpoint_rad_ += clamp_value(
    limited_setpoint - cascade_pitch_setpoint_rad_, -max_step, max_step);

  if (config_.cascade_integral_enable && debug.outer_saturated) {
    cascade_position_integral_m_s_ = clamp_value(
      cascade_position_integral_m_s_ - ex * dt,
      -config_.cascade_integral_limit_m_s, config_.cascade_integral_limit_m_s);
  }

  const double total_trim = config_.manual_trim_rad + auto_trim_rad_;
  const double pitch_reference = total_trim + cascade_pitch_setpoint_rad_;
  const double attitude_error = pitch - pitch_reference;
  debug.pitch_reference_rad = pitch_reference;
  debug.pitch_setpoint_correction_rad = cascade_pitch_setpoint_rad_;
  debug.total_trim_rad = total_trim;
  return config_.cascade_attitude_k_pitch * attitude_error +
    config_.cascade_attitude_k_pitch_rate * pitch_rate;
}

double BalanceController::calculate_lqr(
  const double pitch, const double pitch_rate, const double position_error,
  const double velocity_error, BalanceDebug & debug) const
{
  debug.pitch_reference_rad = config_.manual_trim_rad + auto_trim_rad_;
  debug.total_trim_rad = debug.pitch_reference_rad;
  const double pitch_error = pitch - debug.pitch_reference_rad;
  return -config_.lqr_gain_scale * (
    config_.lqr_k_pitch * pitch_error +
    config_.lqr_k_pitch_rate * pitch_rate +
    config_.lqr_k_position * position_error +
    config_.lqr_k_velocity * velocity_error);
}

double BalanceController::calculate_yaw(
  const double yaw_rate, const double target_yaw_rate, const double common_torque,
  const double torque_limit_each_nm, const double dt, BalanceDebug & debug)
{
  const double filtered = yaw_rate_filter_.update(yaw_rate, dt);
  double accel = 0.0;
  if (yaw_rate_initialized_) {
    accel = (filtered - last_yaw_rate_filtered_rad_s_) / std::max(dt, 1.0e-6);
  }
  last_yaw_rate_filtered_rad_s_ = filtered;
  yaw_rate_initialized_ = true;
  const double filtered_accel = yaw_accel_filter_.update(accel, dt);
  const bool command_active = std::abs(target_yaw_rate) > 1.0e-12;
  const double rate_for_control = command_active ? filtered :
    apply_continuous_deadband(filtered, config_.yaw_rate_deadband_rad_s);
  const double error = target_yaw_rate - rate_for_control;
  const double raw = config_.yaw_enable ? config_.yaw_output_sign * (
    config_.yaw_rate_kp_each_nm_per_rad_s * error -
    config_.yaw_rate_kd_each_nm_per_rad_s2 * filtered_accel) : 0.0;

  const double headroom = std::max(0.0, torque_limit_each_nm - std::abs(common_torque));
  const double allowed = std::min(config_.yaw_torque_limit_each_nm, headroom);
  const double target = clamp_value(raw, -allowed, allowed);
  if (!config_.yaw_enable || (!command_active && std::abs(rate_for_control) < 1.0e-12)) {
    yaw_diff_command_nm_ = 0.0;
  } else {
    const double max_step = config_.yaw_torque_slew_rate_nm_s * dt;
    yaw_diff_command_nm_ += clamp_value(target - yaw_diff_command_nm_, -max_step, max_step);
    yaw_diff_command_nm_ = clamp_value(yaw_diff_command_nm_, -allowed, allowed);
  }
  debug.yaw_differential_each_nm = yaw_diff_command_nm_;
  return yaw_diff_command_nm_;
}

double BalanceController::apply_stiction(
  double total_torque, const double position_error, const double velocity,
  const double pitch_rate, const double target_velocity) const
{
  if (!config_.stiction_enable || std::abs(target_velocity) > 1.0e-6 ||
    std::abs(velocity) > config_.stiction_velocity_limit_mps ||
    std::abs(pitch_rate) > config_.stiction_pitch_rate_limit_rad_s ||
    std::abs(position_error) < config_.stiction_position_error_m ||
    std::abs(total_torque) < config_.stiction_command_min_total_nm)
  {
    return total_torque;
  }
  return total_torque + std::copysign(
    2.0 * config_.stiction_compensation_each_nm, total_torque);
}

void BalanceController::update_auto_trim(
  const double position_error, const double velocity, const double pitch,
  const double pitch_rate, const double target_velocity, const bool outer_saturated,
  const bool torque_saturated, const double dt)
{
  if (!config_.auto_trim_enable || config_.mode != "cascade") {
    auto_trim_stationary_s_ = 0.0;
    return;
  }
  const bool safe = std::abs(target_velocity) < 0.005 &&
    std::abs(velocity) < config_.auto_trim_velocity_limit_mps &&
    std::abs(pitch_rate) < config_.auto_trim_pitch_rate_limit_rad_s &&
    std::abs(pitch) < config_.auto_trim_safe_pitch_rad &&
    std::abs(position_error) < config_.auto_trim_max_position_error_m &&
    !outer_saturated && !torque_saturated;
  if (!safe) {
    auto_trim_stationary_s_ = 0.0;
    return;
  }
  auto_trim_stationary_s_ += dt;
  if (auto_trim_stationary_s_ < config_.auto_trim_dwell_s) {
    return;
  }
  const double corrected_error = apply_continuous_deadband(
    position_error, config_.auto_trim_position_deadband_m);
  double rate = config_.cascade_position_to_pitch_sign *
    config_.auto_trim_gain_rad_per_m_s * corrected_error;
  rate = clamp_value(rate, -config_.auto_trim_max_rate_rad_s, config_.auto_trim_max_rate_rad_s);
  auto_trim_rad_ = clamp_value(
    auto_trim_rad_ + rate * dt, -config_.auto_trim_limit_rad, config_.auto_trim_limit_rad);
}

BalanceOutput BalanceController::prepare_update(const BalanceInput & input)
{
  BalanceOutput out;
  if (!arm_initialized_) {
    arm(input);
  }
  const double dt = clamp_value(input.dt, 1.0e-6, 0.050);
  update_observer(input, out.debug);

  const double forward_raw = clamp_value(input.rc_forward, -1.0, 1.0);
  const double forward_shaped = config_.rc_forward_sign *
    shape_unit_stick(forward_raw, config_.rc_deadband);
  const double requested_velocity = config_.velocity_command_enable ?
    forward_shaped * config_.max_target_velocity_mps : 0.0;
  const double velocity_step = config_.velocity_slew_rate_mps2 * dt;
  target_velocity_mps_ += clamp_value(
    requested_velocity - target_velocity_mps_, -velocity_step, velocity_step);

  const double yaw_raw = clamp_value(input.rc_yaw, -1.0, 1.0);
  const double yaw_shaped = config_.rc_yaw_sign * shape_unit_stick(yaw_raw, config_.rc_deadband);
  const double requested_yaw = config_.yaw_enable ?
    yaw_shaped * config_.max_target_yaw_rate_rad_s : 0.0;
  const double yaw_step = config_.yaw_rate_slew_rate_rad_s2 * dt;
  target_yaw_rate_rad_s_ += clamp_value(
    requested_yaw - target_yaw_rate_rad_s_, -yaw_step, yaw_step);

  target_position_m_ += target_velocity_mps_ * dt;
  out.debug.target_position_m = target_position_m_;
  out.debug.target_velocity_mps = target_velocity_mps_;
  out.debug.position_error_m = out.debug.position_m - target_position_m_;
  out.debug.velocity_error_mps = out.debug.velocity_mps - target_velocity_mps_;

  double total = 0.0;
  if (config_.mode == "cascade") {
    total = calculate_cascade(
      out.debug.pitch_filtered_rad, out.debug.pitch_rate_filtered_rad_s,
      out.debug.position_error_m, out.debug.velocity_error_mps, dt, out.debug);
  } else {
    total = calculate_lqr(
      out.debug.pitch_filtered_rad, out.debug.pitch_rate_filtered_rad_s,
      out.debug.position_error_m, out.debug.velocity_error_mps, out.debug);
  }
  out.debug.model_total_torque_nm = total;
  total = config_.output_gain_sign * total;
  total = apply_stiction(
    total, out.debug.position_error_m, out.debug.velocity_mps,
    out.debug.pitch_rate_filtered_rad_s, target_velocity_mps_);

  const double max_total = 2.0 * config_.torque_limit_each_nm;
  const double limited_total = clamp_value(total, -max_total, max_total);
  out.debug.torque_saturated = std::abs(total - limited_total) > 1.0e-12;
  out.debug.total_torque_limited_nm = limited_total;
  out.debug.common_torque_each_nm = 0.5 * limited_total;
  return out;
}

void BalanceController::finalize_update(
  const BalanceInput & input, const double residual_torque_each_nm,
  const double final_torque_limit_each_nm, BalanceOutput & out)
{
  const double dt = clamp_value(input.dt, 1.0e-6, 0.050);
  const double limit = clamp_value(
    final_torque_limit_each_nm, config_.torque_limit_each_nm,
    config_.hard_torque_limit_each_nm);
  const double baseline_common = out.debug.common_torque_each_nm;
  const double requested_common = baseline_common + residual_torque_each_nm;
  const double combined_common = clamp_value(requested_common, -limit, limit);
  out.debug.residual_torque_requested_each_nm = residual_torque_each_nm;
  out.debug.residual_torque_applied_each_nm = combined_common - baseline_common;
  out.debug.combined_common_torque_each_nm = combined_common;
  out.debug.residual_torque_saturated =
    std::abs(requested_common - combined_common) > 1.0e-12;

  const double yaw_diff = calculate_yaw(
    input.yaw_rate_rad_s, target_yaw_rate_rad_s_, combined_common, limit, dt, out.debug);

  const double left_physical = clamp_value(combined_common + yaw_diff, -limit, limit);
  const double right_physical = clamp_value(combined_common - yaw_diff, -limit, limit);
  out.debug.left_physical_torque_nm = left_physical;
  out.debug.right_physical_torque_nm = right_physical;
  out.left_motor_torque_nm = config_.left_motor_sign * left_physical;
  out.right_motor_torque_nm = config_.right_motor_sign * right_physical;
  out.left_motor_torque_nm = clamp_value(
    out.left_motor_torque_nm, -config_.hard_torque_limit_each_nm,
    config_.hard_torque_limit_each_nm);
  out.right_motor_torque_nm = clamp_value(
    out.right_motor_torque_nm, -config_.hard_torque_limit_each_nm,
    config_.hard_torque_limit_each_nm);
  out.debug.left_motor_command_nm = out.left_motor_torque_nm;
  out.debug.right_motor_command_nm = out.right_motor_torque_nm;

  update_auto_trim(
    out.debug.position_error_m, out.debug.velocity_mps,
    out.debug.pitch_filtered_rad, out.debug.pitch_rate_filtered_rad_s,
    target_velocity_mps_, out.debug.outer_saturated,
    out.debug.torque_saturated || out.debug.residual_torque_saturated, dt);
  out.debug.auto_trim_rad = auto_trim_rad_;
  out.debug.total_trim_rad = config_.manual_trim_rad + auto_trim_rad_;
}

BalanceOutput BalanceController::update(const BalanceInput & input)
{
  BalanceOutput out = prepare_update(input);
  finalize_update(input, 0.0, config_.torque_limit_each_nm, out);
  return out;
}

}  // namespace mujoco_micro
