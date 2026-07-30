#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "custom_msgs/msg/read_djirc.hpp"
#include "custom_msgs/msg/read_dm_motor.hpp"
#include "custom_msgs/msg/write_dm_motor_mit_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "mujoco_micro/control_core.hpp"
#include "mujoco_micro/kinematics.hpp"

namespace mujoco_micro
{
namespace
{
using SteadyTime = std::chrono::steady_clock::time_point;

enum MotorIndex : std::size_t
{
  kLeftJointA = 0,
  kLeftJointB = 1,
  kLeftWheel = 2,
  kRightJointA = 3,
  kRightJointB = 4,
  kRightWheel = 5,
  kMotorCount = 6
};

struct ImuSample
{
  Quaternion orientation{};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
  SteadyTime received{};
  bool valid{false};
};

struct RcSample
{
  bool online{false};
  std::uint8_t right_switch{0};
  double right_y{0.0};
  double left_x{0.0};
  SteadyTime received{};
  bool valid{false};
};

struct MotorSample
{
  bool online{false};
  bool disabled{false};
  bool enabled{false};
  bool overvoltage{false};
  bool undervoltage{false};
  bool overcurrent{false};
  bool mos_overtemperature{false};
  bool rotor_overtemperature{false};
  bool communication_lost{false};
  bool overload{false};
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  SteadyTime received{};
  std::uint64_t sequence{0};
  bool valid{false};

  bool has_fault() const
  {
    return overvoltage || undervoltage || overcurrent || mos_overtemperature ||
           rotor_overtemperature || communication_lost || overload;
  }
};

struct Snapshot
{
  ImuSample imu{};
  RcSample rc{};
  std::array<MotorSample, kMotorCount> motors{};
};

class JointVelocityEstimator
{
public:
  void configure(double cutoff_hz, double nominal_dt)
  {
    filter_.configure(cutoff_hz, nominal_dt);
    initialized_ = false;
  }

  void reset(double position)
  {
    last_position_ = position;
    filter_.reset(0.0);
    initialized_ = true;
  }

  double update(double position, double dt)
  {
    if (!initialized_) {
      reset(position);
      return 0.0;
    }
    const double raw = (position - last_position_) / std::max(dt, 1.0e-6);
    last_position_ = position;
    return filter_.update(raw, dt);
  }

private:
  FirstOrderLowPass filter_{};
  double last_position_{0.0};
  bool initialized_{false};
};

bool sign_is_valid(const double value)
{
  return std::isfinite(value) && std::abs(std::abs(value) - 1.0) < 1.0e-9;
}

}  // namespace

class MujocoMicroNode : public rclcpp::Node
{
public:
  MujocoMicroNode()
  : Node("mujoco_micro")
  {
    load_parameters();
    kinematics_.set_geometry(geometry_);
    balance_.configure(balance_config_);
    for (auto & estimator : joint_velocity_estimators_) {
      estimator.configure(joint_velocity_filter_hz_, control_period_s_);
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        imu_.orientation = {
          msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
        imu_.angular_x = msg->angular_velocity.x;
        imu_.angular_y = msg->angular_velocity.y;
        imu_.angular_z = msg->angular_velocity.z;
        imu_.received = std::chrono::steady_clock::now();
        imu_.valid = true;
      });

    rc_sub_ = create_subscription<custom_msgs::msg::ReadDJIRC>(
      rc_topic_, qos, [this](custom_msgs::msg::ReadDJIRC::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        rc_.online = msg->online != 0U;
        rc_.right_switch = msg->right_switch;
        rc_.right_y = msg->right_y;
        rc_.left_x = msg->left_x;
        rc_.received = std::chrono::steady_clock::now();
        rc_.valid = true;
      });

    for (std::size_t index = 0; index < kMotorCount; ++index) {
      motor_subs_[index] = create_subscription<custom_msgs::msg::ReadDmMotor>(
        motor_read_topics_[index], qos,
        [this, index](custom_msgs::msg::ReadDmMotor::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          auto & sample = motors_[index];
          sample.online = msg->online != 0U;
          sample.disabled = msg->disabled != 0U;
          sample.enabled = msg->enabled != 0U;
          sample.overvoltage = msg->overvoltage != 0U;
          sample.undervoltage = msg->undervoltage != 0U;
          sample.overcurrent = msg->overcurrent != 0U;
          sample.mos_overtemperature = msg->mos_overtemperature != 0U;
          sample.rotor_overtemperature = msg->rotor_overtemperature != 0U;
          sample.communication_lost = msg->communication_lost != 0U;
          sample.overload = msg->overload != 0U;
          sample.position = msg->position;
          sample.velocity = msg->velocity;
          sample.torque = msg->torque;
          sample.received = std::chrono::steady_clock::now();
          sample.sequence++;
          sample.valid = true;
        });
      motor_pubs_[index] = create_publisher<custom_msgs::msg::WriteDmMotorMITControl>(
        motor_write_topics_[index], qos);
    }

    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(debug_topic_, 10);
    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(control_period_s_));
    control_timer_ = create_wall_timer(timer_period, std::bind(&MujocoMicroNode::control_step, this));
    last_control_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "MuJoCo-validated VMC micro controller started at %.3f Hz; dry_run=%s; balance=%s",
      1.0 / control_period_s_, dry_run_ ? "true" : "false",
      balance_config_.mode.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Profile=%s target_B=(%.1f, %.1f)mm Kpitch=%.3f KpitchRate=%.3f "
      "Kpos=%.3f Kvel=%.3f torque_limit_each=%.3fNm",
      profile_name_.c_str(), 1000.0 * vmc_config_.target_x_m,
      1000.0 * vmc_config_.target_y_m, balance_config_.cascade_attitude_k_pitch,
      balance_config_.cascade_attitude_k_pitch_rate,
      balance_config_.cascade_position_kp_rad_per_m,
      balance_config_.cascade_velocity_kd_rad_per_mps,
      balance_config_.torque_limit_each_nm);
    const double equivalent_pendulum_length = std::hypot(
      model_body_com_x_m_ - vmc_config_.target_x_m,
      model_body_com_y_m_ - vmc_config_.target_y_m);
    RCLCPP_INFO(
      get_logger(),
      "Configured equivalent COM-to-wheel length=%.4f m (model parameters are used for gain design)",
      equivalent_pendulum_length);
    RCLCPP_WARN(
      get_logger(),
      "Initial safety state is DISARMED. RC right switch: calibrate=%d, arm=%d, disable=%d",
      calibrate_switch_value_, arm_switch_value_, disable_switch_value_);
  }

  ~MujocoMicroNode() override
  {
    publish_all_disabled();
  }

private:
  template<typename T>
  T parameter(const std::string & name, const T & default_value)
  {
    return declare_parameter<T>(name, default_value);
  }

  void load_parameters()
  {
    control_period_s_ = parameter<double>("control.period_s", 0.003);
    profile_name_ = parameter<std::string>("control.profile_name", "mujoco_validated_hardware");
    dry_run_ = parameter<bool>("control.dry_run", true);
    joint_control_enable_ = parameter<bool>("control.joint_control_enable", true);
    balance_control_enable_ = parameter<bool>("control.balance_control_enable", true);

    imu_topic_ = parameter<std::string>("topics.imu_read", "/ecat/sn1966149/app1/read");
    rc_topic_ = parameter<std::string>("topics.rc_read", "/ecat/sn2228252/app1/read");
    debug_topic_ = parameter<std::string>("topics.debug", "/vmc/debug");
    motor_read_topics_ = {
      parameter<std::string>("topics.left_joint_a_read", "/ecat/sn2228252/app2/read"),
      parameter<std::string>("topics.left_joint_b_read", "/ecat/sn2228252/app3/read"),
      parameter<std::string>("topics.left_wheel_read", "/ecat/sn2228252/app4/read"),
      parameter<std::string>("topics.right_joint_a_read", "/ecat/sn2228252/app5/read"),
      parameter<std::string>("topics.right_joint_b_read", "/ecat/sn2228252/app6/read"),
      parameter<std::string>("topics.right_wheel_read", "/ecat/sn2228252/app7/read")};
    motor_write_topics_ = {
      parameter<std::string>("topics.left_joint_a_write", "/ecat/sn2228252/app2/write"),
      parameter<std::string>("topics.left_joint_b_write", "/ecat/sn2228252/app3/write"),
      parameter<std::string>("topics.left_wheel_write", "/ecat/sn2228252/app4/write"),
      parameter<std::string>("topics.right_joint_a_write", "/ecat/sn2228252/app5/write"),
      parameter<std::string>("topics.right_joint_b_write", "/ecat/sn2228252/app6/write"),
      parameter<std::string>("topics.right_wheel_write", "/ecat/sn2228252/app7/write")};

    geometry_.l1 = parameter<double>("five_bar.l1_m", 0.0804);
    geometry_.l2 = parameter<double>("five_bar.l2_m", 0.1200);
    geometry_.l3 = parameter<double>("five_bar.l3_m", 0.1200);
    geometry_.l4 = parameter<double>("five_bar.l4_m", 0.0804);
    geometry_.l5 = parameter<double>("five_bar.l5_m", 0.0700);
    geometry_.singularity_epsilon = parameter<double>("five_bar.singularity_epsilon", 1.0e-5);
    geometry_.reachability_epsilon = parameter<double>("five_bar.reachability_epsilon", 1.0e-7);

    vmc_config_.target_x_m = parameter<double>("leg.target_x_m", 0.035);
    vmc_config_.target_y_m = parameter<double>("leg.target_y_m", 0.120);
    vmc_config_.total_supported_mass_kg = parameter<double>("vmc.total_supported_mass_kg", 2.8);
    vmc_config_.left_load_fraction = parameter<double>("vmc.left_load_fraction", 0.5);
    vmc_config_.right_load_fraction = parameter<double>("vmc.right_load_fraction", 0.5);
    vmc_config_.kx_n_per_m = parameter<double>("vmc.kx_n_per_m", 10.0);
    vmc_config_.ky_n_per_m = parameter<double>("vmc.ky_n_per_m", 60.0);
    vmc_config_.dx_ns_per_m = parameter<double>("vmc.dx_ns_per_m", 10.0);
    vmc_config_.dy_ns_per_m = parameter<double>("vmc.dy_ns_per_m", 10.0);
    vmc_config_.gravity_compensation = parameter<bool>("vmc.gravity_compensation", true);
    vmc_config_.horizontal_pitch_gravity_compensation = parameter<bool>(
      "vmc.horizontal_pitch_gravity_compensation", false);
    vmc_config_.joint_torque_limit_nm = parameter<double>("vmc.joint_torque_limit_nm", 1.0);

    joint_mit_kp_ = parameter<double>("joint_mit.kp", 7.0);
    joint_mit_kd_ = parameter<double>("joint_mit.kd", 0.28);
    joint_hard_torque_limit_nm_ = parameter<double>("joint_mit.hard_torque_limit_nm", 1.5);
    joint_motor_position_limit_rad_ = parameter<double>("joint_mit.motor_position_limit_rad", 12.5);
    joint_max_runtime_error_rad_ = parameter<double>("joint_mit.max_runtime_joint_error_rad", 1.20);
    // Kept only so old YAML files still load. It no longer blocks joint positioning.
    (void)parameter<double>("joint_mit.max_arm_joint_error_rad", 0.70);
    balance_wait_for_leg_ready_ = parameter<bool>(
      "leg_positioning.balance_wait_for_leg_ready", false);
    leg_ready_joint_error_rad_ = parameter<double>(
      "leg_positioning.ready_joint_error_rad", 0.10);
    leg_ready_joint_velocity_rad_s_ = parameter<double>(
      "leg_positioning.ready_joint_velocity_rad_s", 0.30);
    leg_ready_dwell_s_ = parameter<double>(
      "leg_positioning.ready_dwell_s", 0.30);
    joint_velocity_filter_hz_ = parameter<double>("joint_state.velocity_filter_hz", 25.0);

    calibrations_[kLeftJointA] = load_calibration(
      "joint_calibration.left_a", -0.340847969, 3.50454569, -1.0, 1.0, -1.0);
    calibrations_[kLeftJointB] = load_calibration(
      "joint_calibration.left_b", -0.629243851, 0.899913073, -1.0, 1.0, -1.0);
    calibrations_[kRightJointA] = load_calibration(
      "joint_calibration.right_a", -0.912680626, 2.24167967, 1.0, 1.0, 1.0);
    calibrations_[kRightJointB] = load_calibration(
      "joint_calibration.right_b", -0.602540970, -0.362952948, 1.0, 1.0, 1.0);

    imu_pitch_axis_ = parameter<std::string>("imu.pitch_axis", "pitch");
    imu_pitch_rate_axis_ = parameter<std::string>("imu.pitch_rate_axis", "y");
    imu_yaw_rate_axis_ = parameter<std::string>("imu.yaw_rate_axis", "z");
    imu_pitch_sign_ = parameter<double>("imu.pitch_sign", 1.0);
    imu_pitch_rate_sign_ = parameter<double>("imu.pitch_rate_sign", 1.0);
    imu_yaw_rate_sign_ = parameter<double>("imu.yaw_rate_sign", 1.0);

    require_rc_ = parameter<bool>("safety.require_rc", true);
    imu_timeout_s_ = parameter<double>("safety.imu_timeout_s", 0.05);
    motor_timeout_s_ = parameter<double>("safety.motor_timeout_s", 0.05);
    rc_timeout_s_ = parameter<double>("safety.rc_timeout_s", 0.20);
    arm_max_tilt_rad_ = parameter<double>("safety.arm_max_tilt_deg", 10.0) * kPi / 180.0;
    arm_max_pitch_rate_rad_s_ = parameter<double>("safety.arm_max_pitch_rate_rad_s", 0.30);
    fall_cutoff_rad_ = parameter<double>("safety.fall_cutoff_deg", 25.0) * kPi / 180.0;
    calibrate_switch_value_ = parameter<int>("safety.calibrate_switch_value", 1);
    arm_switch_value_ = parameter<int>("safety.arm_switch_value", 3);
    disable_switch_value_ = parameter<int>("safety.disable_switch_value", 2);

    balance_config_.mode = parameter<std::string>("balance.mode", "cascade");
    balance_config_.control_period_s = control_period_s_;
    balance_config_.wheel_radius_m = parameter<double>("balance.wheel_radius_m", 0.030);
    balance_config_.motor_position_wrap_half_range = parameter<double>(
      "balance.motor_position_wrap_half_range", kPi);
    balance_config_.left_encoder_sign = parameter<double>("balance.left_encoder_sign", 1.0);
    balance_config_.right_encoder_sign = parameter<double>("balance.right_encoder_sign", -1.0);
    balance_config_.pitch_position_compensation_sign = parameter<double>(
      "balance.pitch_position_compensation_sign", 1.0);
    balance_config_.pitch_rate_compensation_sign = parameter<double>(
      "balance.pitch_rate_compensation_sign", 1.0);
    balance_config_.output_gain_sign = parameter<double>("balance.output_gain_sign", -1.0);
    balance_config_.left_motor_sign = parameter<double>("balance.left_motor_sign", -1.0);
    balance_config_.right_motor_sign = parameter<double>("balance.right_motor_sign", 1.0);
    balance_config_.torque_limit_each_nm = parameter<double>("balance.torque_limit_each_nm", 0.20);
    balance_config_.hard_torque_limit_each_nm = parameter<double>(
      "balance.hard_torque_limit_each_nm", 0.45);
    balance_config_.pitch_filter_hz = parameter<double>("balance.pitch_filter_hz", 40.0);
    balance_config_.pitch_rate_filter_hz = parameter<double>("balance.pitch_rate_filter_hz", 25.0);
    balance_config_.wheel_velocity_filter_hz = parameter<double>(
      "balance.wheel_velocity_filter_hz", 30.0);
    balance_config_.position_velocity_filter_hz = parameter<double>(
      "balance.position_velocity_filter_hz", 12.0);
    balance_config_.outer_velocity_filter_hz = parameter<double>(
      "balance.outer_velocity_filter_hz", 8.0);
    balance_config_.velocity_blend = parameter<double>("balance.velocity_blend", 0.0);

    balance_config_.velocity_command_enable = parameter<bool>("rc.velocity_command_enable", false);
    balance_config_.max_target_velocity_mps = parameter<double>("rc.max_target_velocity_mps", 0.20);
    balance_config_.rc_forward_sign = parameter<double>("rc.forward_sign", 1.0);
    balance_config_.rc_deadband = parameter<double>("rc.deadband", 0.08);
    balance_config_.velocity_slew_rate_mps2 = parameter<double>("rc.velocity_slew_rate_mps2", 0.60);
    balance_config_.yaw_enable = parameter<bool>("yaw.enable", false);
    balance_config_.max_target_yaw_rate_rad_s = parameter<double>("yaw.max_target_rate_rad_s", 0.80);
    balance_config_.rc_yaw_sign = parameter<double>("yaw.rc_sign", -1.0);
    balance_config_.yaw_rate_slew_rate_rad_s2 = parameter<double>("yaw.rate_slew_rad_s2", 3.0);
    balance_config_.yaw_rate_kp_each_nm_per_rad_s = parameter<double>("yaw.kp_each_nm_per_rad_s", 0.033);
    balance_config_.yaw_rate_kd_each_nm_per_rad_s2 = parameter<double>("yaw.kd_each_nm_per_rad_s2", 0.0);
    balance_config_.yaw_rate_deadband_rad_s = parameter<double>("yaw.deadband_rad_s", 0.02);
    balance_config_.yaw_rate_filter_hz = parameter<double>("yaw.rate_filter_hz", 15.0);
    balance_config_.yaw_accel_filter_hz = parameter<double>("yaw.accel_filter_hz", 8.0);
    balance_config_.yaw_torque_limit_each_nm = parameter<double>("yaw.torque_limit_each_nm", 0.025);
    balance_config_.yaw_torque_slew_rate_nm_s = parameter<double>("yaw.torque_slew_nm_s", 0.50);
    balance_config_.yaw_output_sign = parameter<double>("yaw.output_sign", 1.0);

    balance_config_.cascade_attitude_k_pitch = parameter<double>("cascade.attitude_k_pitch", 8.0);
    balance_config_.cascade_attitude_k_pitch_rate = parameter<double>("cascade.attitude_k_pitch_rate", 0.06);
    balance_config_.cascade_position_kp_rad_per_m = parameter<double>("cascade.position_kp_rad_per_m", 0.12);
    balance_config_.cascade_velocity_kd_rad_per_mps = parameter<double>("cascade.velocity_kd_rad_per_mps", 0.035);
    balance_config_.cascade_position_ki_rad_per_m_s = parameter<double>("cascade.position_ki_rad_per_m_s", 0.0);
    balance_config_.cascade_integral_enable = parameter<bool>("cascade.integral_enable", false);
    balance_config_.cascade_integral_limit_m_s = parameter<double>("cascade.integral_limit_m_s", 0.20);
    balance_config_.cascade_position_to_pitch_sign = parameter<double>("cascade.position_to_pitch_sign", -1.0);
    balance_config_.cascade_pitch_limit_rad = parameter<double>("cascade.pitch_limit_deg", 8.0) * kPi / 180.0;
    balance_config_.cascade_pitch_slew_rate_rad_s = parameter<double>("cascade.pitch_slew_rate_deg_s", 30.0) * kPi / 180.0;
    balance_config_.cascade_position_error_limit_m = parameter<double>("cascade.position_error_limit_m", 0.50);
    balance_config_.cascade_velocity_error_limit_mps = parameter<double>("cascade.velocity_error_limit_mps", 0.80);
    balance_config_.cascade_arm_bumpless = parameter<bool>("cascade.arm_bumpless", true);
    balance_config_.manual_trim_rad = parameter<double>("cascade.pitch_trim_deg", 0.0) * kPi / 180.0;

    balance_config_.auto_trim_enable = parameter<bool>("auto_trim.enable", false);
    balance_config_.auto_trim_gain_rad_per_m_s = parameter<double>("auto_trim.gain_rad_per_m_s", 0.020);
    balance_config_.auto_trim_limit_rad = parameter<double>("auto_trim.limit_deg", 4.0) * kPi / 180.0;
    balance_config_.auto_trim_position_deadband_m = parameter<double>("auto_trim.position_deadband_m", 0.05);
    balance_config_.auto_trim_velocity_limit_mps = parameter<double>("auto_trim.velocity_limit_mps", 0.25);
    balance_config_.auto_trim_pitch_rate_limit_rad_s = parameter<double>("auto_trim.pitch_rate_limit_rad_s", 0.20);
    balance_config_.auto_trim_safe_pitch_rad = parameter<double>("auto_trim.safe_pitch_deg", 6.0) * kPi / 180.0;
    balance_config_.auto_trim_max_position_error_m = parameter<double>("auto_trim.max_position_error_m", 0.70);
    balance_config_.auto_trim_dwell_s = parameter<double>("auto_trim.dwell_s", 0.25);
    balance_config_.auto_trim_max_rate_rad_s = parameter<double>("auto_trim.max_rate_deg_s", 0.10) * kPi / 180.0;

    balance_config_.stiction_enable = parameter<bool>("stiction.enable", false);
    balance_config_.stiction_velocity_limit_mps = parameter<double>("stiction.velocity_limit_mps", 0.015);
    balance_config_.stiction_pitch_rate_limit_rad_s = parameter<double>("stiction.pitch_rate_limit_rad_s", 0.08);
    balance_config_.stiction_position_error_m = parameter<double>("stiction.position_error_m", 0.008);
    balance_config_.stiction_command_min_total_nm = parameter<double>("stiction.command_min_total_nm", 0.002);
    balance_config_.stiction_compensation_each_nm = parameter<double>("stiction.compensation_each_nm", 0.005);

    balance_config_.lqr_gain_scale = parameter<double>("lqr.gain_scale", 1.0);
    balance_config_.lqr_k_pitch = parameter<double>("lqr.k_pitch", -2.0);
    balance_config_.lqr_k_pitch_rate = parameter<double>("lqr.k_pitch_rate", -0.10);
    balance_config_.lqr_k_position = parameter<double>("lqr.k_position", -0.55);
    balance_config_.lqr_k_velocity = parameter<double>("lqr.k_velocity", -0.13);

    // Retained model parameters for gain generation and future scheduling. They are intentionally
    // declared here so they can be kept in the same YAML even when cascade mode is used.
    model_body_mass_kg_ = parameter<double>("model.body_mass_kg", 2.54);
    model_body_pitch_inertia_kgm2_ = parameter<double>("model.body_pitch_inertia_kgm2", 0.0);
    model_body_com_x_m_ = parameter<double>("model.body_com_x_m", 0.035);
    model_body_com_y_m_ = parameter<double>("model.body_com_y_m", -0.065);
    model_wheel_mass_each_kg_ = parameter<double>("model.wheel_mass_each_kg", 0.13);
    model_wheel_inertia_each_kgm2_ = parameter<double>("model.wheel_inertia_each_kgm2", 0.0);
    model_track_width_m_ = parameter<double>("model.track_width_m", 0.20);

    validate_parameters();
  }

  JointCalibration load_calibration(
    const std::string & prefix, double qm0, double qj0, double motor_sign,
    double ratio, double torque_sign)
  {
    JointCalibration calibration;
    calibration.motor_zero_rad = parameter<double>(prefix + ".motor_zero_rad", qm0);
    calibration.joint_zero_rad = parameter<double>(prefix + ".joint_zero_rad", qj0);
    calibration.motor_sign = parameter<double>(prefix + ".motor_sign", motor_sign);
    calibration.ratio = parameter<double>(prefix + ".ratio", ratio);
    calibration.torque_sign = parameter<double>(prefix + ".torque_sign", torque_sign);
    return calibration;
  }

  void validate_parameters() const
  {
    if (!(control_period_s_ > 0.0)) {
      throw std::runtime_error("control.period_s must be positive");
    }
    if (imu_pitch_axis_ != "roll" && imu_pitch_axis_ != "pitch") {
      throw std::runtime_error("imu.pitch_axis must be roll or pitch");
    }
    if ((imu_pitch_rate_axis_ != "x" && imu_pitch_rate_axis_ != "y" && imu_pitch_rate_axis_ != "z") ||
      (imu_yaw_rate_axis_ != "x" && imu_yaw_rate_axis_ != "y" && imu_yaw_rate_axis_ != "z"))
    {
      throw std::runtime_error("IMU rate axis must be x, y, or z");
    }
    if (!sign_is_valid(imu_pitch_sign_) || !sign_is_valid(imu_pitch_rate_sign_) ||
      !sign_is_valid(imu_yaw_rate_sign_))
    {
      throw std::runtime_error("IMU signs must be exactly +1 or -1");
    }
    for (std::size_t i : {kLeftJointA, kLeftJointB, kRightJointA, kRightJointB}) {
      const auto & c = calibrations_[i];
      if (!sign_is_valid(c.motor_sign) || !sign_is_valid(c.torque_sign) || !(c.ratio > 0.0)) {
        throw std::runtime_error("invalid joint calibration sign or ratio");
      }
    }
    if (vmc_config_.left_load_fraction < 0.0 || vmc_config_.right_load_fraction < 0.0 ||
      std::abs(vmc_config_.left_load_fraction + vmc_config_.right_load_fraction - 1.0) > 1.0e-6)
    {
      throw std::runtime_error("VMC load fractions must be non-negative and sum to 1.0");
    }
    if (!(model_body_mass_kg_ > 0.0) || model_body_pitch_inertia_kgm2_ < 0.0 ||
      !(model_wheel_mass_each_kg_ > 0.0) || model_wheel_inertia_each_kgm2_ < 0.0 ||
      !(model_track_width_m_ > 0.0))
    {
      throw std::runtime_error("invalid physical model mass, inertia, or track width");
    }
  }

  Snapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    Snapshot out;
    out.imu = imu_;
    out.rc = rc_;
    out.motors = motors_;
    return out;
  }

  static double age_seconds(const SteadyTime & now, const SteadyTime & received)
  {
    return std::chrono::duration<double>(now - received).count();
  }

  bool inputs_ready(const Snapshot & s, const SteadyTime & now, std::string & reason) const
  {
    if (!s.imu.valid || age_seconds(now, s.imu.received) > imu_timeout_s_) {
      reason = "IMU missing or timed out";
      return false;
    }
    if (require_rc_ && (!s.rc.valid || !s.rc.online || age_seconds(now, s.rc.received) > rc_timeout_s_)) {
      reason = "RC missing, offline, or timed out";
      return false;
    }
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      const auto & motor = s.motors[i];
      if (!motor.valid || !motor.online || age_seconds(now, motor.received) > motor_timeout_s_) {
        reason = "motor " + std::to_string(i) + " missing, offline, or timed out";
        return false;
      }
      if (motor.has_fault()) {
        reason = "motor " + std::to_string(i) + " reports a fault";
        return false;
      }
      if (!std::isfinite(motor.position) || !std::isfinite(motor.velocity)) {
        reason = "motor " + std::to_string(i) + " has non-finite feedback";
        return false;
      }
    }
    return true;
  }

  double axis_rate(const ImuSample & imu, const std::string & axis) const
  {
    if (axis == "x") {return imu.angular_x;}
    if (axis == "y") {return imu.angular_y;}
    return imu.angular_z;
  }

  bool attitude(const ImuSample & imu, double & pitch, double & pitch_rate, double & yaw_rate) const
  {
    if (!calibrated_) {
      return false;
    }
    Quaternion current = imu.orientation;
    if (!normalize_quaternion(current)) {
      return false;
    }
    const Quaternion relative = relative_quaternion(imu_zero_, current);
    pitch = imu_pitch_sign_ * (
      imu_pitch_axis_ == "roll" ? quaternion_roll(relative) : quaternion_pitch(relative));
    pitch_rate = imu_pitch_rate_sign_ * axis_rate(imu, imu_pitch_rate_axis_);
    yaw_rate = imu_yaw_rate_sign_ * axis_rate(imu, imu_yaw_rate_axis_);
    return std::isfinite(pitch) && std::isfinite(pitch_rate) && std::isfinite(yaw_rate);
  }

  std::array<double, 4> joint_positions(const Snapshot & s) const
  {
    return {
      calibrations_[kLeftJointA].motor_to_joint(s.motors[kLeftJointA].position),
      calibrations_[kLeftJointB].motor_to_joint(s.motors[kLeftJointB].position),
      calibrations_[kRightJointA].motor_to_joint(s.motors[kRightJointA].position),
      calibrations_[kRightJointB].motor_to_joint(s.motors[kRightJointB].position)};
  }

  void calibrate(const Snapshot & s)
  {
    Quaternion zero = s.imu.orientation;
    if (!normalize_quaternion(zero)) {
      RCLCPP_ERROR(get_logger(), "Calibration rejected: invalid IMU quaternion");
      return;
    }
    imu_zero_ = zero;
    calibrated_ = true;
    armed_ = false;
    balance_armed_ = false;
    leg_ready_accumulated_s_ = 0.0;
    arm_transition_required_ = true;
    balance_.calibrate_wheels(
      s.motors[kLeftWheel].position, s.motors[kRightWheel].position);
    const auto q = joint_positions(s);
    for (std::size_t i = 0; i < 4; ++i) {
      joint_velocity_estimators_[i].reset(q[i]);
    }
    publish_all_disabled();
    RCLCPP_INFO(
      get_logger(), "Zero calibrated: IMU, wheel encoders, and joint differentiators reset");
  }

  BalanceInput make_balance_input(
    const Snapshot & s, double pitch, double pitch_rate, double yaw_rate, double dt) const
  {
    BalanceInput input;
    input.pitch_rad = pitch;
    input.pitch_rate_rad_s = pitch_rate;
    input.yaw_rate_rad_s = yaw_rate;
    input.left_wheel_position_rad = s.motors[kLeftWheel].position;
    input.right_wheel_position_rad = s.motors[kRightWheel].position;
    input.left_wheel_velocity_rad_s = s.motors[kLeftWheel].velocity;
    input.right_wheel_velocity_rad_s = s.motors[kRightWheel].velocity;
    input.left_sequence = s.motors[kLeftWheel].sequence;
    input.right_sequence = s.motors[kRightWheel].sequence;
    input.rc_forward = s.rc.right_y;
    input.rc_yaw = s.rc.left_x;
    input.dt = dt;
    return input;
  }

  bool solve_target(
    const std::array<double, 4> & q, IkSolution & left_ik, IkSolution & right_ik) const
  {
    left_ik = kinematics_.inverse(
      vmc_config_.target_x_m, vmc_config_.target_y_m, q[0], q[1]);
    right_ik = kinematics_.inverse(
      vmc_config_.target_x_m, vmc_config_.target_y_m, q[2], q[3]);
    return left_ik.valid && right_ik.valid;
  }

  bool arm(const Snapshot & s, double dt)
  {
    if (!calibrated_) {
      RCLCPP_WARN(get_logger(), "Arm rejected: calibrate first");
      return false;
    }
    double pitch = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate = 0.0;
    if (!attitude(s.imu, pitch, pitch_rate, yaw_rate)) {
      RCLCPP_WARN(get_logger(), "Arm rejected: invalid attitude");
      return false;
    }
    if (std::abs(pitch) > arm_max_tilt_rad_ || std::abs(pitch_rate) > arm_max_pitch_rate_rad_s_) {
      RCLCPP_WARN(
        get_logger(), "Arm rejected: pitch=%+.2f deg, pitch_rate=%+.3f rad/s",
        pitch * 180.0 / kPi, pitch_rate);
      return false;
    }

    const auto q = joint_positions(s);
    IkSolution left_ik;
    IkSolution right_ik;
    if (!solve_target(q, left_ik, right_ik)) {
      RCLCPP_WARN(get_logger(), "Arm rejected: target wheel centre is unreachable");
      return false;
    }
    const std::array<double, 4> desired{left_ik.alpha, left_ik.beta, right_ik.alpha, right_ik.beta};
    double max_joint_error = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
      max_joint_error = std::max(max_joint_error, std::abs(desired[i] - q[i]));
      joint_velocity_estimators_[i].reset(q[i]);
    }

    // Match the original VMC behavior: switch 3 immediately enables the four joint
    // MIT position loops, even when the leg is far from the target wheel centre.
    // Wheel balance is armed separately after the leg reaches the target.
    balance_.reset();
    balance_armed_ = false;
    leg_ready_accumulated_s_ = 0.0;
    if (!balance_wait_for_leg_ready_ && balance_control_enable_) {
      balance_.arm(make_balance_input(s, pitch, pitch_rate, yaw_rate, dt));
      balance_armed_ = true;
    }
    armed_ = true;
    arm_transition_required_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Joint positioning started; max joint error=%.3f rad; target B=(%.1f, %.1f) mm; "
      "balance_wait=%s",
      max_joint_error, 1000.0 * vmc_config_.target_x_m,
      1000.0 * vmc_config_.target_y_m,
      balance_wait_for_leg_ready_ ? "true" : "false");
    return true;
  }

  void disarm(const std::string & reason)
  {
    if (armed_) {
      RCLCPP_WARN(get_logger(), "Controller disarmed: %s", reason.c_str());
    }
    armed_ = false;
    balance_armed_ = false;
    leg_ready_accumulated_s_ = 0.0;
    arm_transition_required_ = true;
    balance_.reset();
    publish_all_disabled();
  }

  custom_msgs::msg::WriteDmMotorMITControl make_mit_command(
    bool enable, double p_des, double v_des, double kp, double kd, double torque,
    double hard_torque_limit) const
  {
    custom_msgs::msg::WriteDmMotorMITControl message;
    message.enable = enable ? 1U : 0U;
    message.p_des = static_cast<float>(clamp_value(
      p_des, -joint_motor_position_limit_rad_, joint_motor_position_limit_rad_));
    message.v_des = static_cast<float>(v_des);
    message.kp = static_cast<float>(kp);
    message.kd = static_cast<float>(kd);
    message.torque = static_cast<float>(clamp_value(
      torque, -std::abs(hard_torque_limit), std::abs(hard_torque_limit)));
    return message;
  }

  void publish_all_disabled()
  {
    for (auto & publisher : motor_pubs_) {
      if (publisher) {
        publisher->publish(make_mit_command(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      }
    }
  }

  void publish_debug(
    double dt, double pitch, double pitch_rate, const std::array<double, 4> & q,
    const std::array<double, 4> & q_des, const FiveBarState & left_state,
    const FiveBarState & right_state, const VmcOutput & left_vmc,
    const VmcOutput & right_vmc, const BalanceDebug & balance_debug,
    const IkSolution & left_ik, const IkSolution & right_ik)
  {
    std_msgs::msg::Float64MultiArray message;
    message.data = {
      armed_ ? 1.0 : 0.0, calibrated_ ? 1.0 : 0.0, dry_run_ ? 1.0 : 0.0,
      balance_armed_ ? 1.0 : 0.0, pitch, pitch_rate, dt,
      left_state.x, left_state.y, left_state.x_dot, left_state.y_dot,
      right_state.x, right_state.y, right_state.x_dot, right_state.y_dot,
      left_state.leg_length, right_state.leg_length,
      left_vmc.fx, left_vmc.fy, left_vmc.tau_a, left_vmc.tau_b,
      right_vmc.fx, right_vmc.fy, right_vmc.tau_a, right_vmc.tau_b,
      q[0], q[1], q[2], q[3], q_des[0], q_des[1], q_des[2], q_des[3],
      balance_debug.position_m, balance_debug.velocity_mps,
      balance_debug.target_position_m, balance_debug.target_velocity_mps,
      balance_debug.position_error_m, balance_debug.velocity_error_mps,
      balance_debug.pitch_reference_rad, balance_debug.model_total_torque_nm,
      balance_debug.total_torque_limited_nm, balance_debug.common_torque_each_nm,
      balance_debug.left_motor_command_nm, balance_debug.right_motor_command_nm,
      balance_debug.yaw_differential_each_nm, balance_debug.auto_trim_rad,
      left_state.jacobian_det, right_state.jacobian_det,
      left_ik.reconstruction_error_m, right_ik.reconstruction_error_m};
    debug_pub_->publish(message);
  }

  void control_step()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(steady_now - last_control_time_).count();
    last_control_time_ = steady_now;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.05) {
      dt = control_period_s_;
    }

    const Snapshot s = snapshot();
    std::string invalid_reason;
    if (!inputs_ready(s, steady_now, invalid_reason)) {
      if (armed_) {
        disarm(invalid_reason);
      } else {
        publish_all_disabled();
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Waiting for valid inputs: %s", invalid_reason.c_str());
      return;
    }

    const int switch_value = static_cast<int>(s.rc.right_switch);
    const bool switch_changed = switch_value != last_switch_value_;
    if (switch_changed && switch_value == calibrate_switch_value_) {
      calibrate(s);
    }
    if (switch_value == disable_switch_value_) {
      if (armed_) {
        disarm("RC disable switch");
      } else {
        publish_all_disabled();
      }
      arm_transition_required_ = true;
    } else if (switch_changed && switch_value == arm_switch_value_) {
      if (arm_transition_required_ || !armed_) {
        (void)arm(s, dt);
      }
    }
    last_switch_value_ = switch_value;

    if (!armed_) {
      publish_all_disabled();
      return;
    }

    double pitch = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate = 0.0;
    if (!attitude(s.imu, pitch, pitch_rate, yaw_rate)) {
      disarm("invalid attitude");
      return;
    }
    if (std::abs(pitch) > fall_cutoff_rad_) {
      disarm("fall angle exceeded");
      return;
    }

    const auto q = joint_positions(s);
    std::array<double, 4> qdot{};
    for (std::size_t i = 0; i < 4; ++i) {
      qdot[i] = joint_velocity_estimators_[i].update(q[i], dt);
    }
    const FiveBarState left_state = kinematics_.forward(q[0], q[1], qdot[0], qdot[1]);
    const FiveBarState right_state = kinematics_.forward(q[2], q[3], qdot[2], qdot[3]);
    if (!left_state.valid || !right_state.valid) {
      disarm("five-bar FK invalid or singular");
      return;
    }

    IkSolution left_ik;
    IkSolution right_ik;
    if (!solve_target(q, left_ik, right_ik)) {
      disarm("five-bar target IK invalid");
      return;
    }
    const std::array<double, 4> q_des{left_ik.alpha, left_ik.beta, right_ik.alpha, right_ik.beta};
    double max_joint_error = 0.0;
    double max_joint_velocity = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
      max_joint_error = std::max(max_joint_error, std::abs(q_des[i] - q[i]));
      max_joint_velocity = std::max(max_joint_velocity, std::abs(qdot[i]));
    }
    // Large initial error is expected during joint positioning. The runtime limit
    // is only a fault after wheel balance has already been engaged.
    if (balance_armed_ && max_joint_error > joint_max_runtime_error_rad_) {
      disarm("joint target error exceeded runtime limit while balancing");
      return;
    }

    const VmcOutput left_vmc = calculate_vmc(
      left_state, pitch,
      vmc_config_.total_supported_mass_kg * vmc_config_.left_load_fraction,
      vmc_config_);
    const VmcOutput right_vmc = calculate_vmc(
      right_state, pitch,
      vmc_config_.total_supported_mass_kg * vmc_config_.right_load_fraction,
      vmc_config_);

    const bool command_enable = !dry_run_;
    const std::array<double, 4> p_des_motor{
      calibrations_[kLeftJointA].joint_to_motor(q_des[0]),
      calibrations_[kLeftJointB].joint_to_motor(q_des[1]),
      calibrations_[kRightJointA].joint_to_motor(q_des[2]),
      calibrations_[kRightJointB].joint_to_motor(q_des[3])};
    const std::array<double, 4> joint_torque{
      calibrations_[kLeftJointA].torque_sign * left_vmc.tau_a,
      calibrations_[kLeftJointB].torque_sign * left_vmc.tau_b,
      calibrations_[kRightJointA].torque_sign * right_vmc.tau_a,
      calibrations_[kRightJointB].torque_sign * right_vmc.tau_b};
    const std::array<std::size_t, 4> joint_indices{
      kLeftJointA, kLeftJointB, kRightJointA, kRightJointB};
    for (std::size_t i = 0; i < 4; ++i) {
      const bool enabled = command_enable && joint_control_enable_;
      motor_pubs_[joint_indices[i]]->publish(make_mit_command(
        enabled, p_des_motor[i], 0.0, enabled ? joint_mit_kp_ : 0.0,
        enabled ? joint_mit_kd_ : 0.0, enabled ? joint_torque[i] : 0.0,
        joint_hard_torque_limit_nm_));
    }

    if (!balance_armed_ && balance_control_enable_) {
      const bool leg_ready =
        max_joint_error <= leg_ready_joint_error_rad_ &&
        max_joint_velocity <= leg_ready_joint_velocity_rad_s_;
      if (leg_ready) {
        leg_ready_accumulated_s_ += dt;
      } else {
        leg_ready_accumulated_s_ = 0.0;
      }
      if (!balance_wait_for_leg_ready_ || leg_ready_accumulated_s_ >= leg_ready_dwell_s_) {
        balance_.arm(make_balance_input(s, pitch, pitch_rate, yaw_rate, dt));
        balance_armed_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Leg target reached; wheel balance armed. max_error=%.4f rad max_velocity=%.4f rad/s",
          max_joint_error, max_joint_velocity);
      }
    }

    BalanceOutput balance_output;
    if (balance_control_enable_ && balance_armed_) {
      balance_output = balance_.update(make_balance_input(s, pitch, pitch_rate, yaw_rate, dt));
    }
    const bool wheel_enable = command_enable && balance_control_enable_ && balance_armed_;
    motor_pubs_[kLeftWheel]->publish(make_mit_command(
      wheel_enable, 0.0, 0.0, 0.0, 0.0,
      wheel_enable ? balance_output.left_motor_torque_nm : 0.0,
      balance_config_.hard_torque_limit_each_nm));
    motor_pubs_[kRightWheel]->publish(make_mit_command(
      wheel_enable, 0.0, 0.0, 0.0, 0.0,
      wheel_enable ? balance_output.right_motor_torque_nm : 0.0,
      balance_config_.hard_torque_limit_each_nm));

    publish_debug(
      dt, pitch, pitch_rate, q, q_des, left_state, right_state, left_vmc,
      right_vmc, balance_output.debug, left_ik, right_ik);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 200,
      "state=%s pitch=%+.2fdeg max_qerr=%.3frad x=%.4fm v=%+.4fm/s "
      "B_L=(%.1f,%.1f)mm B_R=(%.1f,%.1f)mm wheel_tau=(%+.3f,%+.3f)Nm",
      balance_armed_ ? "BALANCE" : "LEG_POSITIONING",
      pitch * 180.0 / kPi, max_joint_error, balance_output.debug.position_m,
      balance_output.debug.velocity_mps, left_state.x * 1000.0, left_state.y * 1000.0,
      right_state.x * 1000.0, right_state.y * 1000.0,
      balance_output.left_motor_torque_nm, balance_output.right_motor_torque_nm);
  }

  mutable std::mutex data_mutex_;
  ImuSample imu_{};
  RcSample rc_{};
  std::array<MotorSample, kMotorCount> motors_{};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<custom_msgs::msg::ReadDJIRC>::SharedPtr rc_sub_;
  std::array<rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr, kMotorCount>
    motor_subs_{};
  std::array<rclcpp::Publisher<custom_msgs::msg::WriteDmMotorMITControl>::SharedPtr, kMotorCount>
    motor_pubs_{};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string imu_topic_;
  std::string rc_topic_;
  std::string debug_topic_;
  std::array<std::string, kMotorCount> motor_read_topics_{};
  std::array<std::string, kMotorCount> motor_write_topics_{};

  FiveBarGeometry geometry_{};
  FiveBarKinematics kinematics_{};
  VmcConfig vmc_config_{};
  BalanceConfig balance_config_{};
  BalanceController balance_{};
  std::array<JointCalibration, kMotorCount> calibrations_{};
  std::array<JointVelocityEstimator, 4> joint_velocity_estimators_{};

  double control_period_s_{0.003};
  std::string profile_name_{"mujoco_validated_hardware"};
  bool dry_run_{true};
  bool joint_control_enable_{true};
  bool balance_control_enable_{true};
  double joint_mit_kp_{7.0};
  double joint_mit_kd_{0.28};
  double joint_hard_torque_limit_nm_{1.5};
  double joint_motor_position_limit_rad_{12.5};
  double joint_max_runtime_error_rad_{1.20};
  bool balance_wait_for_leg_ready_{false};
  double leg_ready_joint_error_rad_{0.10};
  double leg_ready_joint_velocity_rad_s_{0.30};
  double leg_ready_dwell_s_{0.30};
  double joint_velocity_filter_hz_{25.0};

  std::string imu_pitch_axis_{"pitch"};
  std::string imu_pitch_rate_axis_{"y"};
  std::string imu_yaw_rate_axis_{"z"};
  double imu_pitch_sign_{1.0};
  double imu_pitch_rate_sign_{1.0};
  double imu_yaw_rate_sign_{1.0};

  bool require_rc_{true};
  double imu_timeout_s_{0.05};
  double motor_timeout_s_{0.05};
  double rc_timeout_s_{0.20};
  double arm_max_tilt_rad_{10.0 * kPi / 180.0};
  double arm_max_pitch_rate_rad_s_{0.30};
  double fall_cutoff_rad_{25.0 * kPi / 180.0};
  int calibrate_switch_value_{1};
  int arm_switch_value_{3};
  int disable_switch_value_{2};

  double model_body_mass_kg_{2.54};
  double model_body_pitch_inertia_kgm2_{0.0};
  double model_body_com_x_m_{0.035};
  double model_body_com_y_m_{-0.065};
  double model_wheel_mass_each_kg_{0.13};
  double model_wheel_inertia_each_kgm2_{0.0};
  double model_track_width_m_{0.20};

  Quaternion imu_zero_{};
  bool calibrated_{false};
  bool armed_{false};
  bool balance_armed_{false};
  double leg_ready_accumulated_s_{0.0};
  bool arm_transition_required_{true};
  int last_switch_value_{-1};
  SteadyTime last_control_time_{};
};

}  // namespace mujoco_micro

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mujoco_micro::MujocoMicroNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco_micro"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
