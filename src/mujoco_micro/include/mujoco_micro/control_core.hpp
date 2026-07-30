#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "mujoco_micro/kinematics.hpp"

namespace mujoco_micro
{

constexpr double kPi = 3.141592653589793238462643383279502884;

inline double clamp_value(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

inline double apply_continuous_deadband(const double value, const double deadband)
{
  const double magnitude = std::abs(value);
  if (magnitude <= deadband) {
    return 0.0;
  }
  return std::copysign(magnitude - deadband, value);
}

inline double shape_unit_stick(const double value, const double deadband)
{
  const double clamped = clamp_value(value, -1.0, 1.0);
  return apply_continuous_deadband(clamped, deadband) /
         std::max(1.0 - deadband, 1.0e-9);
}

class FirstOrderLowPass
{
public:
  void configure(double cutoff_hz, double nominal_dt)
  {
    cutoff_hz_ = std::max(0.0, cutoff_hz);
    nominal_dt_ = std::max(1.0e-6, nominal_dt);
    initialized_ = false;
    value_ = 0.0;
  }

  void reset(double value = 0.0)
  {
    value_ = value;
    initialized_ = true;
  }

  double update(double input, double actual_dt)
  {
    if (!initialized_) {
      reset(input);
      return input;
    }
    if (cutoff_hz_ <= 0.0) {
      value_ = input;
      return value_;
    }
    double dt = nominal_dt_;
    if (std::isfinite(actual_dt) && actual_dt > 1.0e-6) {
      dt = clamp_value(actual_dt, 1.0e-6, 0.050);
    }
    const double tau = 1.0 / (2.0 * kPi * cutoff_hz_);
    const double alpha = dt / (tau + dt);
    value_ += alpha * (input - value_);
    return value_;
  }

  double value() const noexcept {return value_;}

private:
  double cutoff_hz_{0.0};
  double nominal_dt_{0.003};
  double value_{0.0};
  bool initialized_{false};
};

class WrappedAngleUnwrapper
{
public:
  void configure(double half_range)
  {
    half_range_ = std::max(half_range, 1.0e-6);
    period_ = 2.0 * half_range_;
  }

  void reset(double raw_angle)
  {
    last_raw_ = raw_angle;
    unwrapped_ = 0.0;
    initialized_ = true;
  }

  double update(double raw_angle)
  {
    if (!initialized_) {
      reset(raw_angle);
      return 0.0;
    }
    double delta = raw_angle - last_raw_;
    while (delta > half_range_) {delta -= period_;}
    while (delta < -half_range_) {delta += period_;}
    unwrapped_ += delta;
    last_raw_ = raw_angle;
    return unwrapped_;
  }

private:
  double half_range_{kPi};
  double period_{2.0 * kPi};
  double last_raw_{0.0};
  double unwrapped_{0.0};
  bool initialized_{false};
};

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

bool normalize_quaternion(Quaternion & q);
Quaternion relative_quaternion(const Quaternion & reference, Quaternion current);
double quaternion_roll(const Quaternion & q);
double quaternion_pitch(const Quaternion & q);

struct JointCalibration
{
  double motor_zero_rad{0.0};
  double joint_zero_rad{0.0};
  double motor_sign{1.0};
  double ratio{1.0};
  double torque_sign{1.0};

  double motor_to_joint(double motor_position) const;
  double joint_to_motor(double joint_position) const;
};

struct VmcConfig
{
  double target_x_m{0.035};
  double target_y_m{0.120};
  double total_supported_mass_kg{2.8};
  double left_load_fraction{0.5};
  double right_load_fraction{0.5};
  double kx_n_per_m{10.0};
  double ky_n_per_m{60.0};
  double dx_ns_per_m{10.0};
  double dy_ns_per_m{10.0};
  bool gravity_compensation{true};
  bool horizontal_pitch_gravity_compensation{false};
  double joint_torque_limit_nm{1.0};
};

struct VmcOutput
{
  double fx{0.0};
  double fy{0.0};
  double tau_a{0.0};
  double tau_b{0.0};
};

VmcOutput calculate_vmc(
  const FiveBarState & state, double pitch, double supported_mass,
  const VmcConfig & config);

struct BalanceConfig
{
  std::string mode{"cascade"};
  double control_period_s{0.003};
  double wheel_radius_m{0.030};
  double motor_position_wrap_half_range{kPi};
  double left_encoder_sign{1.0};
  double right_encoder_sign{-1.0};
  double pitch_position_compensation_sign{1.0};
  double pitch_rate_compensation_sign{1.0};
  double output_gain_sign{-1.0};
  double left_motor_sign{-1.0};
  double right_motor_sign{1.0};
  double torque_limit_each_nm{0.20};
  double hard_torque_limit_each_nm{0.45};

  double pitch_filter_hz{40.0};
  double pitch_rate_filter_hz{25.0};
  double wheel_velocity_filter_hz{30.0};
  double position_velocity_filter_hz{12.0};
  double outer_velocity_filter_hz{8.0};
  double velocity_blend{0.0};

  bool velocity_command_enable{false};
  double max_target_velocity_mps{0.20};
  double rc_forward_sign{1.0};
  double rc_deadband{0.08};
  double velocity_slew_rate_mps2{0.60};

  bool yaw_enable{false};
  double max_target_yaw_rate_rad_s{0.80};
  double rc_yaw_sign{-1.0};
  double yaw_rate_slew_rate_rad_s2{3.0};
  double yaw_rate_kp_each_nm_per_rad_s{0.033};
  double yaw_rate_kd_each_nm_per_rad_s2{0.0};
  double yaw_rate_deadband_rad_s{0.02};
  double yaw_rate_filter_hz{15.0};
  double yaw_accel_filter_hz{8.0};
  double yaw_torque_limit_each_nm{0.025};
  double yaw_torque_slew_rate_nm_s{0.50};
  double yaw_output_sign{1.0};

  double cascade_attitude_k_pitch{8.0};
  double cascade_attitude_k_pitch_rate{0.06};
  double cascade_position_kp_rad_per_m{0.12};
  double cascade_velocity_kd_rad_per_mps{0.035};
  double cascade_position_ki_rad_per_m_s{0.0};
  bool cascade_integral_enable{false};
  double cascade_integral_limit_m_s{0.20};
  double cascade_position_to_pitch_sign{-1.0};
  double cascade_pitch_limit_rad{8.0 * kPi / 180.0};
  double cascade_pitch_slew_rate_rad_s{30.0 * kPi / 180.0};
  double cascade_position_error_limit_m{0.50};
  double cascade_velocity_error_limit_mps{0.80};
  bool cascade_arm_bumpless{true};
  double manual_trim_rad{0.0};

  bool auto_trim_enable{false};
  double auto_trim_gain_rad_per_m_s{0.020};
  double auto_trim_limit_rad{4.0 * kPi / 180.0};
  double auto_trim_position_deadband_m{0.05};
  double auto_trim_velocity_limit_mps{0.25};
  double auto_trim_pitch_rate_limit_rad_s{0.20};
  double auto_trim_safe_pitch_rad{6.0 * kPi / 180.0};
  double auto_trim_max_position_error_m{0.70};
  double auto_trim_dwell_s{0.25};
  double auto_trim_max_rate_rad_s{0.10 * kPi / 180.0};

  bool stiction_enable{false};
  double stiction_velocity_limit_mps{0.015};
  double stiction_pitch_rate_limit_rad_s{0.08};
  double stiction_position_error_m{0.008};
  double stiction_command_min_total_nm{0.002};
  double stiction_compensation_each_nm{0.005};

  double lqr_gain_scale{1.0};
  double lqr_k_pitch{-2.0};
  double lqr_k_pitch_rate{-0.10};
  double lqr_k_position{-0.55};
  double lqr_k_velocity{-0.13};
};

struct BalanceInput
{
  double pitch_rad{0.0};
  double pitch_rate_rad_s{0.0};
  double yaw_rate_rad_s{0.0};
  double left_wheel_position_rad{0.0};
  double right_wheel_position_rad{0.0};
  double left_wheel_velocity_rad_s{0.0};
  double right_wheel_velocity_rad_s{0.0};
  std::uint64_t left_sequence{0};
  std::uint64_t right_sequence{0};
  double rc_forward{0.0};
  double rc_yaw{0.0};
  double dt{0.003};
};

struct BalanceDebug
{
  double position_m{0.0};
  double velocity_mps{0.0};
  double target_position_m{0.0};
  double target_velocity_mps{0.0};
  double position_error_m{0.0};
  double velocity_error_mps{0.0};
  double pitch_filtered_rad{0.0};
  double pitch_rate_filtered_rad_s{0.0};
  double pitch_reference_rad{0.0};
  double pitch_setpoint_correction_rad{0.0};
  double total_trim_rad{0.0};
  double model_total_torque_nm{0.0};
  double total_torque_limited_nm{0.0};
  double common_torque_each_nm{0.0};
  double yaw_differential_each_nm{0.0};
  double left_physical_torque_nm{0.0};
  double right_physical_torque_nm{0.0};
  double left_motor_command_nm{0.0};
  double right_motor_command_nm{0.0};
  double velocity_motor_based_mps{0.0};
  double velocity_position_fd_mps{0.0};
  double auto_trim_rad{0.0};
  bool torque_saturated{false};
  bool outer_saturated{false};
};

struct BalanceOutput
{
  double left_motor_torque_nm{0.0};
  double right_motor_torque_nm{0.0};
  BalanceDebug debug{};
};

class BalanceController
{
public:
  void configure(const BalanceConfig & config);
  const BalanceConfig & config() const noexcept {return config_;}

  void calibrate_wheels(double left_raw_position, double right_raw_position);
  void arm(const BalanceInput & input);
  void reset();
  BalanceOutput update(const BalanceInput & input);

  bool armed_state_initialized() const noexcept {return arm_initialized_;}

private:
  double calculate_cascade(
    double pitch, double pitch_rate, double position_error, double velocity_error,
    double dt, BalanceDebug & debug);
  double calculate_lqr(
    double pitch, double pitch_rate, double position_error, double velocity_error,
    BalanceDebug & debug) const;
  double calculate_yaw(
    double yaw_rate, double target_yaw_rate, double common_torque, double dt,
    BalanceDebug & debug);
  double apply_stiction(
    double total_torque, double position_error, double velocity, double pitch_rate,
    double target_velocity) const;
  void update_auto_trim(
    double position_error, double velocity, double pitch, double pitch_rate,
    double target_velocity, bool outer_saturated, bool torque_saturated, double dt);
  void update_observer(const BalanceInput & input, BalanceDebug & debug);

  BalanceConfig config_{};
  WrappedAngleUnwrapper left_unwrapper_{};
  WrappedAngleUnwrapper right_unwrapper_{};
  FirstOrderLowPass pitch_filter_{};
  FirstOrderLowPass pitch_rate_filter_{};
  FirstOrderLowPass wheel_velocity_filter_{};
  FirstOrderLowPass position_velocity_filter_{};
  FirstOrderLowPass outer_velocity_filter_{};
  FirstOrderLowPass yaw_rate_filter_{};
  FirstOrderLowPass yaw_accel_filter_{};

  bool arm_initialized_{false};
  bool fd_initialized_{false};
  std::uint64_t last_left_sequence_{0};
  std::uint64_t last_right_sequence_{0};
  double last_position_m_{0.0};
  double velocity_from_position_mps_{0.0};
  double fd_elapsed_s_{0.0};
  double target_position_m_{0.0};
  double target_velocity_mps_{0.0};
  double target_yaw_rate_rad_s_{0.0};
  double cascade_pitch_setpoint_rad_{0.0};
  double cascade_position_integral_m_s_{0.0};
  double auto_trim_rad_{0.0};
  double auto_trim_stationary_s_{0.0};
  double last_yaw_rate_filtered_rad_s_{0.0};
  bool yaw_rate_initialized_{false};
  double yaw_diff_command_nm_{0.0};
};

}  // namespace mujoco_micro
