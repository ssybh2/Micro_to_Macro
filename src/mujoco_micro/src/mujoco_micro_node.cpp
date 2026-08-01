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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "custom_msgs/msg/read_djirc.hpp"
#include "custom_msgs/msg/read_dm_motor.hpp"
#include "custom_msgs/msg/write_dm_motor_mit_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "mujoco_micro/control_core.hpp"
#include "mujoco_micro/kinematics.hpp"
#include "mujoco_micro/policy_runner.hpp"

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
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  bool orientation_available{false};
  bool linear_acceleration_available{false};
  SteadyTime received{};
  bool valid{false};
};

struct RcSample
{
  bool online{false};
  std::uint8_t right_switch{0};
  double right_y{0.0};
  double right_x{0.0};
  double left_x{0.0};
  double left_y{0.0};
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

struct RollControlOutput
{
  double roll_filtered_rad{0.0};
  double roll_rate_filtered_rad_s{0.0};
  double target_roll_rad{0.0};
  double target_roll_command_rad{0.0};
  double error_rad{0.0};
  // Signed total wheel-centre height difference: left_target_y - right_target_y.
  double leg_difference_m{0.0};
  double left_target_y_m{0.0};
  double right_target_y_m{0.0};
};

struct HeightControlOutput
{
  double rc_left_y{0.0};
  double command_m{0.125};
  double target_m{0.125};
  double target_rate_mps{0.0};
  double measured_m{0.125};
  double measured_rate_mps{0.0};
};

struct PolicyControlOutput
{
  std::array<float, PolicyRunner::kObservationSize> observation{};
  double previous_action{0.0};
  double action{0.0};
  double residual_torque_nm{0.0};
  double inference_time_us{0.0};
  bool active{false};
};

struct RecoveryControlOutput
{
  std::array<float, RecoveryPolicyRunner::kObservationSize> observation{};
  std::array<double, RecoveryPolicyRunner::kActionSize> action{};
  std::array<double, 4> joint_target{};
  double measured_height_m{0.0};
  double measured_height_rate_mps{0.0};
  double height_target_m{0.0};
  double height_target_rate_mps{0.0};
  double measured_leg_angle_rad{0.0};
  double leg_angle_target_rad{0.0};
  double gravity_alignment_error_rad{0.0};
  double wheel_position_m{0.0};
  double wheel_velocity_mps{0.0};
  double wheel_effort_nm{0.0};
  double inference_time_us{0.0};
  double success_dwell_s{0.0};
  bool extension_unlocked{false};
  bool active{false};
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

double wrap_to_pi(double angle)
{
  while (angle > kPi) {angle -= 2.0 * kPi;}
  while (angle < -kPi) {angle += 2.0 * kPi;}
  return angle;
}

double nearest_equivalent_angle(double angle, double reference)
{
  return angle + 2.0 * kPi * std::round((reference - angle) / (2.0 * kPi));
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
    roll_filter_.configure(roll_angle_filter_hz_, control_period_s_);
    roll_rate_filter_.configure(roll_rate_filter_hz_, control_period_s_);
    gravity_x_filter_.configure(imu_gravity_filter_hz_, control_period_s_);
    gravity_y_filter_.configure(imu_gravity_filter_hz_, control_period_s_);
    gravity_z_filter_.configure(imu_gravity_filter_hz_, control_period_s_);
    recovery_pitch_rate_filter_.configure(recovery_pitch_rate_filter_hz_, control_period_s_);
    recovery_left_wheel_unwrapper_.configure(balance_config_.motor_position_wrap_half_range);
    recovery_right_wheel_unwrapper_.configure(balance_config_.motor_position_wrap_half_range);
    if (policy_enable_) {
      if (policy_model_path_.empty()) {
        policy_model_path_ = ament_index_cpp::get_package_share_directory("mujoco_micro") +
          "/models/policy.onnx";
      }
      policy_.load(policy_model_path_);
    }
    if (recovery_enable_) {
      if (recovery_model_path_.empty()) {
        recovery_model_path_ = ament_index_cpp::get_package_share_directory("mujoco_micro") +
          "/models/recovery_policy.onnx";
      }
      recovery_policy_.load(recovery_model_path_);
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        imu_.orientation = {
          msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
        imu_.orientation_available = msg->orientation_covariance[0] != -1.0;
        imu_.angular_x = msg->angular_velocity.x;
        imu_.angular_y = msg->angular_velocity.y;
        imu_.angular_z = msg->angular_velocity.z;
        imu_.linear_x = msg->linear_acceleration.x;
        imu_.linear_y = msg->linear_acceleration.y;
        imu_.linear_z = msg->linear_acceleration.z;
        imu_.linear_acceleration_available = msg->linear_acceleration_covariance[0] != -1.0;
        imu_.received = std::chrono::steady_clock::now();
        imu_.valid = true;
      });

    rc_sub_ = create_subscription<custom_msgs::msg::ReadDJIRC>(
      rc_topic_, qos, [this](custom_msgs::msg::ReadDJIRC::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        rc_.online = msg->online != 0U;
        rc_.right_switch = msg->right_switch;
        rc_.right_y = msg->right_y;
        rc_.right_x = msg->right_x;
        rc_.left_x = msg->left_x;
        rc_.left_y = msg->left_y;
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
    RCLCPP_INFO(
      get_logger(),
      "Roll control=%s RC=right_x max_target=%.2fdeg max_leg_difference=%.1fmm",
      roll_control_enable_ ? "enabled" : "disabled",
      roll_max_target_rad_ * 180.0 / kPi, 1000.0 * roll_max_leg_difference_m_);
    RCLCPP_INFO(
      get_logger(),
      "Height RC=left_y range=[%.1f, %.1f]mm center=%.1fmm slew=%.1fmm/s; policy=%s%s",
      1000.0 * height_min_m_, 1000.0 * height_max_m_, 1000.0 * height_center_m_,
      1000.0 * height_target_slew_rate_mps_, policy_enable_ ? "enabled: " : "disabled",
      policy_enable_ ? policy_model_path_.c_str() : "");
    RCLCPP_INFO(
      get_logger(),
      "Recovery=%s%s policy=%.1fHz goal=%.1fmm start/fail pitch=%.1f/%.1fdeg "
      "handoff=(wheel=%.3fs,joint=%.3fs)",
      recovery_enable_ ? "enabled: " : "disabled",
      recovery_enable_ ? recovery_model_path_.c_str() : "",
      1.0 / recovery_policy_period_s_, 1000.0 * recovery_goal_height_m_,
      recovery_start_max_pitch_rad_ * 180.0 / kPi,
      recovery_terminate_pitch_rad_ * 180.0 / kPi,
      recovery_wheel_handoff_blend_s_, recovery_joint_handoff_blend_s_);
    RCLCPP_INFO(
      get_logger(),
      "Attitude reference=%s gravity_source=%s mount_trim=(roll=%+.2f,pitch=%+.2f)deg",
      imu_reference_mode_.c_str(), imu_gravity_source_.c_str(),
      imu_mount_roll_rad_ * 180.0 / kPi, imu_mount_pitch_rad_ * 180.0 / kPi);
    if (imu_reference_mode_ == "gravity") {
      RCLCPP_WARN(
        get_logger(),
        "Initial state DISARMED. RC switch: optional motion-origin reset=%d, arm=%d, disable=%d; "
        "attitude zero comes from gravity",
        calibrate_switch_value_, arm_switch_value_, disable_switch_value_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Initial state DISARMED. RC switch: relative-attitude calibrate=%d, arm=%d, disable=%d",
        calibrate_switch_value_, arm_switch_value_, disable_switch_value_);
    }
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

    height_control_enable_ = parameter<bool>("height.enable", true);
    height_center_m_ = parameter<double>("height.center_m", 0.125);
    height_min_m_ = parameter<double>("height.min_m", 0.090);
    height_max_m_ = parameter<double>("height.max_m", 0.160);
    height_target_slew_rate_mps_ = parameter<double>("height.target_slew_rate_mps", 0.060);
    height_rc_deadband_ = parameter<double>("height.rc_deadband", 0.08);
    height_rc_sign_ = parameter<double>("height.rc_sign", 1.0);

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
    imu_roll_axis_ = parameter<std::string>("imu.roll_axis", "roll");
    imu_roll_rate_axis_ = parameter<std::string>("imu.roll_rate_axis", "x");
    imu_yaw_rate_axis_ = parameter<std::string>("imu.yaw_rate_axis", "z");
    imu_pitch_sign_ = parameter<double>("imu.pitch_sign", 1.0);
    imu_pitch_rate_sign_ = parameter<double>("imu.pitch_rate_sign", 1.0);
    imu_roll_sign_ = parameter<double>("imu.roll_sign", 1.0);
    imu_roll_rate_sign_ = parameter<double>("imu.roll_rate_sign", 1.0);
    imu_yaw_rate_sign_ = parameter<double>("imu.yaw_rate_sign", 1.0);
    imu_reference_mode_ = parameter<std::string>("imu.reference_mode", "gravity");
    imu_gravity_source_ = parameter<std::string>("imu.gravity_source", "orientation");
    imu_mount_roll_rad_ = parameter<double>("imu.mount_roll_deg", 0.0) * kPi / 180.0;
    imu_mount_pitch_rad_ = parameter<double>("imu.mount_pitch_deg", 0.0) * kPi / 180.0;
    imu_gravity_accel_sign_ = parameter<double>("imu.gravity_accel_sign", 1.0);
    imu_gravity_filter_hz_ = parameter<double>("imu.gravity_filter_hz", 10.0);
    imu_gravity_norm_min_mps2_ = parameter<double>("imu.gravity_norm_min_mps2", 7.8);
    imu_gravity_norm_max_mps2_ = parameter<double>("imu.gravity_norm_max_mps2", 11.8);
    imu_gravity_ready_dwell_s_ = parameter<double>("imu.gravity_ready_dwell_s", 0.20);
    imu_gravity_timeout_s_ = parameter<double>("imu.gravity_timeout_s", 0.20);

    require_rc_ = parameter<bool>("safety.require_rc", true);
    imu_timeout_s_ = parameter<double>("safety.imu_timeout_s", 0.05);
    motor_timeout_s_ = parameter<double>("safety.motor_timeout_s", 0.05);
    rc_timeout_s_ = parameter<double>("safety.rc_timeout_s", 0.20);
    arm_max_tilt_rad_ = parameter<double>("safety.arm_max_tilt_deg", 10.0) * kPi / 180.0;
    arm_max_pitch_rate_rad_s_ = parameter<double>("safety.arm_max_pitch_rate_rad_s", 0.30);
    fall_cutoff_rad_ = parameter<double>("safety.fall_cutoff_deg", 25.0) * kPi / 180.0;
    arm_max_roll_rad_ = parameter<double>("safety.arm_max_roll_deg", 10.0) * kPi / 180.0;
    arm_max_roll_rate_rad_s_ = parameter<double>("safety.arm_max_roll_rate_rad_s", 0.40);
    fall_cutoff_roll_rad_ =
      parameter<double>("safety.fall_cutoff_roll_deg", 25.0) * kPi / 180.0;
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

    roll_control_enable_ = parameter<bool>("roll.enable", false);
    roll_trim_rad_ = parameter<double>("roll.trim_deg", 0.0) * kPi / 180.0;
    roll_kp_leg_difference_m_per_rad_ =
      parameter<double>("roll.kp_leg_difference_m_per_rad", 0.080);
    roll_kd_leg_difference_m_per_rad_s_ =
      parameter<double>("roll.kd_leg_difference_m_per_rad_s", 0.008);
    roll_max_leg_difference_m_ = parameter<double>("roll.max_leg_difference_m", 0.016);
    roll_leg_difference_slew_rate_mps_ =
      parameter<double>("roll.leg_difference_slew_rate_mps", 0.040);
    roll_max_target_rad_ =
      parameter<double>("roll.max_target_roll_deg", 5.0) * kPi / 180.0;
    roll_target_slew_rate_rad_s_ =
      parameter<double>("roll.target_slew_rate_deg_s", 20.0) * kPi / 180.0;
    roll_rc_deadband_ = parameter<double>("roll.rc_deadband", 0.08);
    roll_rc_sign_ = parameter<double>("roll.rc_sign", 1.0);
    roll_output_sign_ = parameter<double>("roll.output_sign", 1.0);
    roll_angle_filter_hz_ = parameter<double>("roll.angle_filter_hz", 15.0);
    roll_rate_filter_hz_ = parameter<double>("roll.rate_filter_hz", 10.0);

    policy_enable_ = parameter<bool>("policy.enable", true);
    policy_model_path_ = parameter<std::string>("policy.model_path", "");
    policy_residual_scale_nm_ = parameter<double>("policy.residual_scale_nm", 0.060);
    policy_final_torque_limit_each_nm_ = parameter<double>(
      "policy.final_torque_limit_each_nm", 0.260);

    recovery_enable_ = parameter<bool>("recovery.enable", true);
    recovery_model_path_ = parameter<std::string>("recovery.model_path", "");
    recovery_policy_period_s_ = parameter<double>("recovery.policy_period_s", 0.010);
    recovery_start_max_pitch_rad_ =
      parameter<double>("recovery.start_max_pitch_deg", 35.0) * kPi / 180.0;
    recovery_start_max_pitch_rate_rad_s_ =
      parameter<double>("recovery.start_max_pitch_rate_rad_s", 4.0);
    recovery_terminate_pitch_rad_ =
      parameter<double>("recovery.terminate_pitch_deg", 70.0) * kPi / 180.0;
    recovery_max_duration_s_ = parameter<double>("recovery.max_duration_s", 8.0);
    recovery_max_wheel_travel_m_ = parameter<double>("recovery.max_wheel_travel_m", 0.80);
    recovery_wheel_torque_limit_nm_ =
      parameter<double>("recovery.wheel_torque_limit_nm", 0.45);
    recovery_wheel_output_sign_ = parameter<double>("recovery.wheel_output_sign", 1.0);
    recovery_wheel_torque_slew_nm_s_ =
      parameter<double>("recovery.wheel_torque_slew_nm_s", 2.0);
    recovery_height_min_m_ = parameter<double>("recovery.height_min_m", 0.012);
    recovery_height_max_m_ = parameter<double>("recovery.height_max_m", 0.130);
    recovery_goal_height_m_ = parameter<double>("recovery.goal_height_m", 0.120);
    recovery_height_rate_center_mps_ =
      parameter<double>("recovery.height_rate_center_mps", 0.040);
    recovery_height_rate_span_mps_ =
      parameter<double>("recovery.height_rate_span_mps", 0.100);
    recovery_height_rate_min_mps_ =
      parameter<double>("recovery.height_rate_min_mps", -0.040);
    recovery_height_rate_max_mps_ =
      parameter<double>("recovery.height_rate_max_mps", 0.140);
    recovery_leg_angle_limit_rad_ =
      parameter<double>("recovery.leg_angle_limit_deg", 35.0) * kPi / 180.0;
    recovery_leg_angle_rate_rad_s_ =
      parameter<double>("recovery.leg_angle_rate_deg_s", 120.0) * kPi / 180.0;
    recovery_tilt_unlock_min_height_m_ =
      parameter<double>("recovery.tilt_unlock_min_height_m", 0.020);
    recovery_tilt_unlock_full_height_m_ =
      parameter<double>("recovery.tilt_unlock_full_height_m", 0.070);
    recovery_alignment_unlock_rad_ =
      parameter<double>("recovery.alignment_unlock_deg", 5.0) * kPi / 180.0;
    recovery_alignment_relock_rad_ =
      parameter<double>("recovery.alignment_relock_deg", 8.0) * kPi / 180.0;
    recovery_alignment_unlock_dwell_s_ =
      parameter<double>("recovery.alignment_unlock_dwell_s", 0.10);
    recovery_success_pitch_rad_ =
      parameter<double>("recovery.success_pitch_deg", 5.0) * kPi / 180.0;
    recovery_success_alignment_rad_ =
      parameter<double>("recovery.success_alignment_deg", 5.0) * kPi / 180.0;
    recovery_success_pitch_rate_rad_s_ =
      parameter<double>("recovery.success_pitch_rate_rad_s", 0.35);
    recovery_success_height_tolerance_m_ =
      parameter<double>("recovery.success_height_tolerance_m", 0.010);
    recovery_success_wheel_velocity_mps_ =
      parameter<double>("recovery.success_wheel_velocity_mps", 0.30);
    recovery_success_dwell_s_ = parameter<double>("recovery.success_dwell_s", 0.15);
    recovery_pitch_rate_filter_hz_ =
      parameter<double>("recovery.pitch_rate_filter_hz", 1.675);
    recovery_joint_kp_ = parameter<double>("recovery.joint_kp", 7.0);
    recovery_joint_kd_ = parameter<double>("recovery.joint_kd", 0.28);
    recovery_joint_torque_limit_nm_ =
      parameter<double>("recovery.joint_torque_limit_nm", 1.5);
    recovery_left_alpha0_ = parameter<double>("recovery.left_alpha0", 2.4430524286);
    recovery_left_beta0_ = parameter<double>("recovery.left_beta0", 0.6977234730);
    recovery_right_alpha0_ = parameter<double>("recovery.right_alpha0", 2.4438691429);
    recovery_right_beta0_ = parameter<double>("recovery.right_beta0", 0.6985402266);
    recovery_hip_pivot_height_m_ =
      parameter<double>("recovery.hip_pivot_height_m", 0.1530);
    recovery_chassis_com_height_m_ =
      parameter<double>("recovery.chassis_com_height_m", 0.1895);
    // Keep the legacy parameter as the default for old configuration files, but split the
    // transition because frozen recovery wheel effort and recovery leg geometry need very
    // different time scales.  Wheel authority must move to NORMAL quickly; changing the leg
    // geometry too quickly moves the COM and can throw the robot out of NORMAL's capture region.
    const double legacy_handoff_blend_s =
      parameter<double>("recovery.handoff_blend_s", 0.10);
    recovery_wheel_handoff_blend_s_ =
      parameter<double>("recovery.wheel_handoff_blend_s", legacy_handoff_blend_s);
    recovery_joint_handoff_blend_s_ =
      parameter<double>("recovery.joint_handoff_blend_s", legacy_handoff_blend_s);

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
    if ((imu_pitch_axis_ != "roll" && imu_pitch_axis_ != "pitch") ||
      (imu_roll_axis_ != "roll" && imu_roll_axis_ != "pitch"))
    {
      throw std::runtime_error("imu pitch/roll axes must be roll or pitch");
    }
    if (imu_reference_mode_ != "relative" && imu_reference_mode_ != "gravity") {
      throw std::runtime_error("imu.reference_mode must be relative or gravity");
    }
    if (imu_gravity_source_ != "orientation" && imu_gravity_source_ != "accelerometer") {
      throw std::runtime_error("imu.gravity_source must be orientation or accelerometer");
    }
    const auto rate_axis_valid = [](const std::string & axis) {
        return axis == "x" || axis == "y" || axis == "z";
      };
    if (!rate_axis_valid(imu_pitch_rate_axis_) || !rate_axis_valid(imu_roll_rate_axis_) ||
      !rate_axis_valid(imu_yaw_rate_axis_))
    {
      throw std::runtime_error("IMU rate axis must be x, y, or z");
    }
    if (!sign_is_valid(imu_pitch_sign_) || !sign_is_valid(imu_pitch_rate_sign_) ||
      !sign_is_valid(imu_roll_sign_) || !sign_is_valid(imu_roll_rate_sign_) ||
      !sign_is_valid(imu_yaw_rate_sign_) || !sign_is_valid(roll_rc_sign_) ||
      !sign_is_valid(roll_output_sign_) || !sign_is_valid(height_rc_sign_) ||
      !sign_is_valid(imu_gravity_accel_sign_))
    {
      throw std::runtime_error("IMU/roll signs must be exactly +1 or -1");
    }
    if (!std::isfinite(imu_mount_roll_rad_) || !std::isfinite(imu_mount_pitch_rad_) ||
      imu_gravity_filter_hz_ < 0.0 || !(imu_gravity_norm_min_mps2_ > 0.0) ||
      !(imu_gravity_norm_max_mps2_ > imu_gravity_norm_min_mps2_) ||
      imu_gravity_ready_dwell_s_ < 0.0 || !(imu_gravity_timeout_s_ > 0.0))
    {
      throw std::runtime_error("invalid IMU gravity-reference parameter");
    }
    if (roll_kp_leg_difference_m_per_rad_ < 0.0 ||
      roll_kd_leg_difference_m_per_rad_s_ < 0.0 ||
      roll_max_leg_difference_m_ < 0.0 || roll_leg_difference_slew_rate_mps_ < 0.0 ||
      roll_max_target_rad_ < 0.0 || roll_target_slew_rate_rad_s_ < 0.0 ||
      roll_rc_deadband_ < 0.0 || roll_rc_deadband_ >= 1.0 ||
      roll_angle_filter_hz_ < 0.0 || roll_rate_filter_hz_ < 0.0)
    {
      throw std::runtime_error("invalid roll controller gain, limit, filter, or deadband");
    }
    if (!(height_min_m_ > 0.0) || !(height_min_m_ < height_center_m_) ||
      !(height_center_m_ < height_max_m_) || !(height_target_slew_rate_mps_ > 0.0) ||
      height_rc_deadband_ < 0.0 || height_rc_deadband_ >= 1.0)
    {
      throw std::runtime_error("invalid height range, slew rate, or RC deadband");
    }
    if (!(policy_residual_scale_nm_ > 0.0) ||
      policy_final_torque_limit_each_nm_ < balance_config_.torque_limit_each_nm ||
      policy_final_torque_limit_each_nm_ > balance_config_.hard_torque_limit_each_nm)
    {
      throw std::runtime_error(
              "policy residual scale/final torque limit is invalid or exceeds the hard limit");
    }
    if (!(recovery_policy_period_s_ > 0.0) || !(recovery_start_max_pitch_rad_ > 0.0) ||
      recovery_start_max_pitch_rad_ > recovery_terminate_pitch_rad_ ||
      recovery_start_max_pitch_rate_rad_s_ < 0.0 ||
      !(recovery_terminate_pitch_rad_ < kPi) || !(recovery_max_duration_s_ > 0.0) ||
      !(recovery_max_wheel_travel_m_ > 0.0) ||
      !(recovery_wheel_torque_limit_nm_ > 0.0) ||
      recovery_wheel_torque_limit_nm_ > balance_config_.hard_torque_limit_each_nm ||
      !sign_is_valid(recovery_wheel_output_sign_) ||
      !(recovery_wheel_torque_slew_nm_s_ > 0.0) ||
      !(recovery_height_min_m_ > 0.0) ||
      !(recovery_height_min_m_ < recovery_goal_height_m_) ||
      !(recovery_goal_height_m_ < recovery_height_max_m_) ||
      !(recovery_height_rate_min_mps_ < recovery_height_rate_max_mps_) ||
      recovery_height_rate_span_mps_ < 0.0 || !(recovery_leg_angle_limit_rad_ > 0.0) ||
      !(recovery_leg_angle_rate_rad_s_ > 0.0) ||
      !(recovery_tilt_unlock_min_height_m_ < recovery_tilt_unlock_full_height_m_) ||
      !(recovery_alignment_unlock_rad_ < recovery_alignment_relock_rad_) ||
      recovery_alignment_unlock_dwell_s_ < 0.0 || !(recovery_success_dwell_s_ > 0.0) ||
      recovery_pitch_rate_filter_hz_ < 0.0 || recovery_joint_kp_ < 0.0 ||
      recovery_joint_kd_ < 0.0 || !(recovery_joint_torque_limit_nm_ > 0.0) ||
      recovery_joint_torque_limit_nm_ > joint_hard_torque_limit_nm_ ||
      !(recovery_chassis_com_height_m_ >= recovery_hip_pivot_height_m_) ||
      recovery_wheel_handoff_blend_s_ < 0.0 || recovery_joint_handoff_blend_s_ < 0.0)
    {
      throw std::runtime_error("invalid recovery policy, gate, geometry, or safety parameter");
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

  bool update_gravity_reference(const ImuSample & imu, double dt)
  {
    gravity_accel_norm_mps2_ = std::hypot(
      imu.linear_x, std::hypot(imu.linear_y, imu.linear_z));
    if (imu_reference_mode_ != "gravity") {
      return calibrated_;
    }

    Vector3 candidate{};
    bool valid = false;
    if (imu_gravity_source_ == "orientation" && imu.orientation_available) {
      Quaternion orientation = imu.orientation;
      if (normalize_quaternion(orientation)) {
        candidate = world_up_axis_in_body(orientation);
        valid = normalize_vector(candidate);
      }
    } else if (imu_gravity_source_ == "accelerometer" &&
      imu.linear_acceleration_available &&
      std::isfinite(gravity_accel_norm_mps2_) &&
      gravity_accel_norm_mps2_ >= imu_gravity_norm_min_mps2_ &&
      gravity_accel_norm_mps2_ <= imu_gravity_norm_max_mps2_)
    {
      const double scale = imu_gravity_accel_sign_ / gravity_accel_norm_mps2_;
      candidate = {scale * imu.linear_x, scale * imu.linear_y, scale * imu.linear_z};
      if (!gravity_filter_initialized_) {
        gravity_x_filter_.reset(candidate.x);
        gravity_y_filter_.reset(candidate.y);
        gravity_z_filter_.reset(candidate.z);
        gravity_filter_initialized_ = true;
      } else {
        candidate.x = gravity_x_filter_.update(candidate.x, dt);
        candidate.y = gravity_y_filter_.update(candidate.y, dt);
        candidate.z = gravity_z_filter_.update(candidate.z, dt);
      }
      valid = normalize_vector(candidate);
    }

    if (valid) {
      gravity_up_body_ = candidate;
      gravity_invalid_elapsed_s_ = 0.0;
      gravity_ready_accumulated_s_ += dt;
      if (!gravity_reference_ready_ &&
        gravity_ready_accumulated_s_ >= imu_gravity_ready_dwell_s_)
      {
        gravity_reference_ready_ = true;
        calibrated_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Gravity attitude reference ready: source=%s up_body=(%+.4f,%+.4f,%+.4f) "
          "accel_norm=%.3fm/s^2",
          imu_gravity_source_.c_str(), gravity_up_body_.x, gravity_up_body_.y,
          gravity_up_body_.z, gravity_accel_norm_mps2_);
      }
    } else {
      gravity_ready_accumulated_s_ = 0.0;
      gravity_invalid_elapsed_s_ += dt;
      if (gravity_reference_ready_ && gravity_invalid_elapsed_s_ > imu_gravity_timeout_s_) {
        gravity_reference_ready_ = false;
        gravity_filter_initialized_ = false;
        calibrated_ = false;
        RCLCPP_ERROR(get_logger(), "Gravity attitude reference lost");
      }
    }
    return gravity_reference_ready_;
  }

  bool attitude_reference_ready() const
  {
    return imu_reference_mode_ == "gravity" ? gravity_reference_ready_ : calibrated_;
  }

  bool attitude(
    const ImuSample & imu, double & roll, double & roll_rate,
    double & pitch, double & pitch_rate, double & yaw_rate) const
  {
    if (!attitude_reference_ready()) {
      return false;
    }
    double reference_roll = 0.0;
    double reference_pitch = 0.0;
    if (imu_reference_mode_ == "gravity") {
      reference_roll = gravity_roll(gravity_up_body_);
      reference_pitch = gravity_pitch(gravity_up_body_);
    } else {
      Quaternion current = imu.orientation;
      if (!imu.orientation_available || !normalize_quaternion(current)) {
        return false;
      }
      const Quaternion relative = relative_quaternion(imu_zero_, current);
      reference_roll = quaternion_roll(relative);
      reference_pitch = quaternion_pitch(relative);
    }
    roll = imu_roll_sign_ * (
      imu_roll_axis_ == "pitch" ? reference_pitch : reference_roll);
    pitch = imu_pitch_sign_ * (
      imu_pitch_axis_ == "roll" ? reference_roll : reference_pitch);
    if (imu_reference_mode_ == "gravity") {
      roll -= imu_mount_roll_rad_;
      pitch -= imu_mount_pitch_rad_;
    }
    roll_rate = imu_roll_rate_sign_ * axis_rate(imu, imu_roll_rate_axis_);
    pitch_rate = imu_pitch_rate_sign_ * axis_rate(imu, imu_pitch_rate_axis_);
    yaw_rate = imu_yaw_rate_sign_ * axis_rate(imu, imu_yaw_rate_axis_);
    return std::isfinite(roll) && std::isfinite(roll_rate) &&
           std::isfinite(pitch) && std::isfinite(pitch_rate) && std::isfinite(yaw_rate);
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
    if (imu_reference_mode_ == "relative") {
      Quaternion zero = s.imu.orientation;
      if (!s.imu.orientation_available || !normalize_quaternion(zero)) {
        RCLCPP_ERROR(get_logger(), "Calibration rejected: invalid IMU quaternion");
        return;
      }
      imu_zero_ = zero;
      calibrated_ = true;
    } else if (!gravity_reference_ready_) {
      RCLCPP_WARN(
        get_logger(),
        "Motion origins reset, but gravity attitude reference is not ready yet");
    }
    armed_ = false;
    balance_armed_ = false;
    recovery_active_ = false;
    normal_handoff_active_ = false;
    leg_ready_accumulated_s_ = 0.0;
    arm_transition_required_ = true;
    balance_.calibrate_wheels(
      s.motors[kLeftWheel].position, s.motors[kRightWheel].position);
    reset_roll_controller(0.0, 0.0);
    reset_policy_state();
    const auto q = joint_positions(s);
    for (std::size_t i = 0; i < 4; ++i) {
      joint_velocity_estimators_[i].reset(q[i]);
    }
    publish_all_disabled();
    RCLCPP_INFO(
      get_logger(),
      imu_reference_mode_ == "relative" ?
      "Relative IMU zero, wheel encoders, and joint differentiators reset" :
      "Wheel encoders and joint differentiators reset; gravity attitude zero is unchanged");
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

  double height_command_from_rc(double rc_left_y) const
  {
    if (!height_control_enable_) {
      return vmc_config_.target_y_m;
    }
    const double shaped = height_rc_sign_ * shape_unit_stick(rc_left_y, height_rc_deadband_);
    const double span = shaped >= 0.0 ?
      height_max_m_ - height_center_m_ : height_center_m_ - height_min_m_;
    return clamp_value(height_center_m_ + shaped * span, height_min_m_, height_max_m_);
  }

  static double leg_length_rate(const FiveBarState & state, double pivot_midpoint_x)
  {
    if (!state.valid || state.leg_length <= 1.0e-9) {
      return 0.0;
    }
    const double relative_x = state.x - pivot_midpoint_x;
    return (relative_x * state.x_dot + state.y * state.y_dot) / state.leg_length;
  }

  void reset_height_controller(double measured_height_m, double rc_left_y)
  {
    height_command_m_ = height_command_from_rc(rc_left_y);
    height_target_m_ = clamp_value(measured_height_m, height_min_m_, height_max_m_);
    height_target_rate_mps_ = 0.0;
  }

  HeightControlOutput update_height_controller(
    double rc_left_y, const FiveBarState & left_state, const FiveBarState & right_state,
    double dt)
  {
    HeightControlOutput out;
    out.rc_left_y = clamp_value(rc_left_y, -1.0, 1.0);
    out.measured_m = 0.5 * (left_state.leg_length + right_state.leg_length);
    const double pivot_midpoint_x = 0.5 * geometry_.l5;
    out.measured_rate_mps = 0.5 * (
      leg_length_rate(left_state, pivot_midpoint_x) +
      leg_length_rate(right_state, pivot_midpoint_x));

    height_command_m_ = height_command_from_rc(out.rc_left_y);
    const double old_target = height_target_m_;
    const double max_step = height_target_slew_rate_mps_ * dt;
    height_target_m_ += clamp_value(height_command_m_ - height_target_m_, -max_step, max_step);
    height_target_m_ = clamp_value(height_target_m_, height_min_m_, height_max_m_);
    height_target_rate_mps_ = (height_target_m_ - old_target) / std::max(dt, 1.0e-6);

    out.command_m = height_command_m_;
    out.target_m = height_target_m_;
    out.target_rate_mps = height_target_rate_mps_;
    return out;
  }

  void reset_policy_state()
  {
    policy_previous_action_ = 0.0;
  }

  std::array<float, PolicyRunner::kObservationSize> make_policy_observation(
    double pitch, double pitch_rate, const HeightControlOutput & height,
    const BalanceDebug & balance_debug) const
  {
    return {
      static_cast<float>(pitch / 0.35),
      static_cast<float>(pitch_rate / 2.0),
      static_cast<float>(balance_debug.policy_wheel_position_m / 0.50),
      static_cast<float>(balance_debug.policy_wheel_velocity_mps / 1.0),
      static_cast<float>((height.measured_m - 0.125) / 0.035),
      static_cast<float>(height.measured_rate_mps / 0.08),
      static_cast<float>((height.command_m - 0.125) / 0.035),
      static_cast<float>(height.target_rate_mps / 0.06),
      static_cast<float>((height.target_m - height.measured_m) / 0.04),
      static_cast<float>(balance_debug.common_torque_each_nm / 0.20),
      static_cast<float>(policy_previous_action_)};
  }

  void reset_roll_controller(double roll, double roll_rate)
  {
    roll_filter_.reset(roll);
    roll_rate_filter_.reset(roll_rate);
    roll_target_command_rad_ = 0.0;
    roll_leg_difference_m_ = 0.0;
  }

  RollControlOutput update_roll_controller(
    double roll, double roll_rate, double rc_right_x, double base_height_m,
    double dt, bool active)
  {
    RollControlOutput out;
    out.roll_filtered_rad = roll_filter_.update(roll, dt);
    out.roll_rate_filtered_rad_s = roll_rate_filter_.update(roll_rate, dt);

    const double shaped = roll_rc_sign_ * shape_unit_stick(rc_right_x, roll_rc_deadband_);
    const double requested_command =
      (active && roll_control_enable_) ? shaped * roll_max_target_rad_ : 0.0;
    const double target_step = roll_target_slew_rate_rad_s_ * dt;
    roll_target_command_rad_ += clamp_value(
      requested_command - roll_target_command_rad_, -target_step, target_step);

    out.target_roll_command_rad = roll_target_command_rad_;
    out.target_roll_rad = roll_trim_rad_ + roll_target_command_rad_;
    out.error_rad = out.target_roll_rad - out.roll_filtered_rad;

    double requested_difference = 0.0;
    if (active && roll_control_enable_) {
      requested_difference = roll_output_sign_ * (
        roll_kp_leg_difference_m_per_rad_ * out.error_rad -
        roll_kd_leg_difference_m_per_rad_s_ * out.roll_rate_filtered_rad_s);
      requested_difference = clamp_value(
        requested_difference, -roll_max_leg_difference_m_, roll_max_leg_difference_m_);
    }

    const double available_height = std::max(
      0.0, std::min(base_height_m - height_min_m_, height_max_m_ - base_height_m));
    const double effective_difference_limit = std::min(
      roll_max_leg_difference_m_, 2.0 * available_height);
    requested_difference = clamp_value(
      requested_difference, -effective_difference_limit, effective_difference_limit);

    const double difference_step = roll_leg_difference_slew_rate_mps_ * dt;
    roll_leg_difference_m_ += clamp_value(
      requested_difference - roll_leg_difference_m_, -difference_step, difference_step);
    roll_leg_difference_m_ = clamp_value(
      roll_leg_difference_m_, -effective_difference_limit, effective_difference_limit);

    out.leg_difference_m = roll_leg_difference_m_;
    out.left_target_y_m = base_height_m + 0.5 * roll_leg_difference_m_;
    out.right_target_y_m = base_height_m - 0.5 * roll_leg_difference_m_;
    return out;
  }

  std::array<double, 4> recovery_model_joint_positions(
    const std::array<double, 4> & q) const
  {
    return {
      recovery_left_alpha0_ - q[0], recovery_left_beta0_ - q[1],
      recovery_right_alpha0_ - q[2], recovery_right_beta0_ - q[3]};
  }

  double recovery_mean_leg_angle(
    const FiveBarState & left_state, const FiveBarState & right_state) const
  {
    const double offset =
      0.5 * (left_state.x + right_state.x) - 0.5 * geometry_.l5;
    const double down = 0.5 * (left_state.y + right_state.y);
    return std::atan2(offset, down);
  }

  double recovery_alignment_error(
    double pitch, const FiveBarState & left_state, const FiveBarState & right_state) const
  {
    const double offset = 0.5 * (left_state.x + right_state.x) - 0.5 * geometry_.l5;
    const double down = 0.5 * (left_state.y + right_state.y);
    const double com_down = down +
      (recovery_chassis_com_height_m_ - recovery_hip_pivot_height_m_);
    return wrap_to_pi(pitch + std::atan2(offset, com_down));
  }

  void initialize_recovery(
    const Snapshot & s, const std::array<double, 4> & q,
    const FiveBarState & left_state, const FiveBarState & right_state,
    double pitch_rate)
  {
    recovery_left_wheel_unwrapper_.reset(s.motors[kLeftWheel].position);
    recovery_right_wheel_unwrapper_.reset(s.motors[kRightWheel].position);
    recovery_height_target_m_ = clamp_value(
      0.5 * (left_state.leg_length + right_state.leg_length),
      recovery_height_min_m_, recovery_height_max_m_);
    recovery_height_target_rate_mps_ = 0.0;
    recovery_last_measured_height_m_ =
      0.5 * (left_state.leg_length + right_state.leg_length);
    recovery_leg_angle_target_rad_ = recovery_mean_leg_angle(left_state, right_state);
    recovery_previous_action_.fill(0.0);
    recovery_joint_target_ = q;
    recovery_wheel_effort_nm_ = 0.0;
    recovery_requested_wheel_effort_nm_ = 0.0;
    recovery_policy_phase_s_ = 0.0;
    recovery_observation_elapsed_s_ = 0.0;
    recovery_alignment_dwell_s_ = 0.0;
    recovery_success_dwell_accumulated_s_ = 0.0;
    recovery_elapsed_s_ = 0.0;
    recovery_extension_unlocked_ = false;
    recovery_pitch_rate_filter_.reset(pitch_rate);
    recovery_debug_ = RecoveryControlOutput{};
    recovery_debug_.active = true;
    recovery_active_ = true;
    normal_handoff_active_ = false;
    balance_armed_ = false;
    leg_ready_accumulated_s_ = 0.0;
    balance_.calibrate_wheels(
      s.motors[kLeftWheel].position, s.motors[kRightWheel].position);
    reset_policy_state();
    reset_roll_controller(0.0, 0.0);
    for (std::size_t i = 0; i < q.size(); ++i) {
      joint_velocity_estimators_[i].reset(q[i]);
    }
  }

  std::array<float, RecoveryPolicyRunner::kObservationSize> make_recovery_observation(
    double pitch, double pitch_rate, double wheel_position_m, double wheel_velocity_mps,
    double measured_height_m, double measured_height_rate_mps,
    double measured_leg_angle_rad, double alignment_error_rad,
    const std::array<double, 4> & q, const std::array<double, 4> & qdot) const
  {
    const auto model_q = recovery_model_joint_positions(q);
    const std::array<double, 4> model_qdot{-qdot[0], -qdot[1], -qdot[2], -qdot[3]};
    std::array<float, RecoveryPolicyRunner::kObservationSize> observation{};
    observation[0] = static_cast<float>(pitch / 0.60);
    observation[1] = static_cast<float>(pitch_rate / 4.0);
    observation[2] = static_cast<float>(wheel_position_m / 0.50);
    observation[3] = static_cast<float>(wheel_velocity_mps / 1.50);
    observation[4] = static_cast<float>((measured_height_m - 0.070) / 0.055);
    observation[5] = static_cast<float>(measured_height_rate_mps / 0.15);
    observation[6] = static_cast<float>((recovery_height_target_m_ - 0.070) / 0.055);
    observation[7] = static_cast<float>((recovery_goal_height_m_ - measured_height_m) / 0.105);
    observation[8] = static_cast<float>(measured_leg_angle_rad / 0.60);
    observation[9] = static_cast<float>(alignment_error_rad / 0.60);
    for (std::size_t i = 0; i < 4; ++i) {
      observation[10 + i] = static_cast<float>(model_q[i] / 0.70);
      observation[14 + i] = static_cast<float>(model_qdot[i] / 5.0);
    }
    observation[18] = static_cast<float>(recovery_wheel_effort_nm_ / 0.45);
    observation[19] = static_cast<float>(recovery_height_target_rate_mps_ / 0.14);
    for (std::size_t i = 0; i < recovery_previous_action_.size(); ++i) {
      observation[20 + i] = static_cast<float>(recovery_previous_action_[i]);
    }
    observation[23] = recovery_extension_unlocked_ ? 1.0F : 0.0F;
    return observation;
  }

  bool solve_recovery_target(
    const std::array<double, 4> & q, std::array<double, 4> & q_des,
    IkSolution & left_ik, IkSolution & right_ik) const
  {
    const double unlock = clamp_value(
      (recovery_height_target_m_ - recovery_tilt_unlock_min_height_m_) /
      (recovery_tilt_unlock_full_height_m_ - recovery_tilt_unlock_min_height_m_),
      0.0, 1.0);
    const double safe_angle = recovery_leg_angle_target_rad_ * unlock;
    const double target_x = 0.5 * geometry_.l5 +
      recovery_height_target_m_ * std::sin(safe_angle);
    const double target_y = recovery_height_target_m_ * std::cos(safe_angle);
    left_ik = kinematics_.inverse(target_x, target_y, q[0], q[1]);
    right_ik = kinematics_.inverse(target_x, target_y, q[2], q[3]);
    if (!left_ik.valid || !right_ik.valid) {
      return false;
    }
    q_des = {left_ik.alpha, left_ik.beta, right_ik.alpha, right_ik.beta};
    for (std::size_t i = 0; i < q_des.size(); ++i) {
      // FiveBarKinematics returns principal angles in [-pi, pi]. Recovery
      // trajectories at very short leg lengths can cross that branch cut;
      // command the equivalent angle nearest the measured continuous joint.
      q_des[i] = nearest_equivalent_angle(q_des[i], q[i]);
    }
    return true;
  }

  void handoff_recovery_to_normal(
    const Snapshot & s, double pitch, double pitch_rate, double yaw_rate,
    double measured_height_m, double roll, double roll_rate, double dt)
  {
    recovery_active_ = false;
    normal_handoff_active_ =
      recovery_wheel_handoff_blend_s_ > 0.0 || recovery_joint_handoff_blend_s_ > 0.0;
    normal_handoff_elapsed_s_ = 0.0;
    normal_handoff_wheel_effort_nm_ = recovery_wheel_effort_nm_;
    normal_handoff_joint_target_ = recovery_joint_target_;
    reset_height_controller(measured_height_m, s.rc.left_y);
    balance_.calibrate_wheels(
      s.motors[kLeftWheel].position, s.motors[kRightWheel].position);
    balance_.arm(make_balance_input(s, pitch, pitch_rate, yaw_rate, dt));
    balance_armed_ = true;
    reset_policy_state();
    reset_roll_controller(roll, roll_rate);
    RCLCPP_WARN(
      get_logger(),
      "RECOVERY success; handing off to NORMAL at pitch=%+.2fdeg height=%.1fmm "
      "leg_target=%+.2fdeg wheel_effort=%+.3fNm blends=(wheel=%.3fs,joint=%.3fs)",
      pitch * 180.0 / kPi, 1000.0 * measured_height_m,
      recovery_leg_angle_target_rad_ * 180.0 / kPi,
      normal_handoff_wheel_effort_nm_, recovery_wheel_handoff_blend_s_,
      recovery_joint_handoff_blend_s_);
  }

  bool solve_target(
    const std::array<double, 4> & q, double left_target_y_m, double right_target_y_m,
    IkSolution & left_ik, IkSolution & right_ik) const
  {
    left_ik = kinematics_.inverse(
      vmc_config_.target_x_m, left_target_y_m, q[0], q[1]);
    right_ik = kinematics_.inverse(
      vmc_config_.target_x_m, right_target_y_m, q[2], q[3]);
    return left_ik.valid && right_ik.valid;
  }

  bool arm(const Snapshot & s, double dt)
  {
    if (!attitude_reference_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "Arm rejected: %s attitude reference is not ready",
        imu_reference_mode_.c_str());
      return false;
    }
    double roll = 0.0;
    double roll_rate = 0.0;
    double pitch = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate = 0.0;
    if (!attitude(s.imu, roll, roll_rate, pitch, pitch_rate, yaw_rate)) {
      RCLCPP_WARN(get_logger(), "Arm rejected: invalid attitude");
      return false;
    }
    const double arm_pitch_limit = recovery_enable_ ?
      recovery_start_max_pitch_rad_ : arm_max_tilt_rad_;
    const double arm_pitch_rate_limit = recovery_enable_ ?
      recovery_start_max_pitch_rate_rad_s_ : arm_max_pitch_rate_rad_s_;
    if (std::abs(pitch) > arm_pitch_limit ||
      std::abs(pitch_rate) > arm_pitch_rate_limit ||
      std::abs(roll) > arm_max_roll_rad_ ||
      std::abs(roll_rate) > arm_max_roll_rate_rad_s_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Arm rejected: pitch=%+.2fdeg pitch_rate=%+.3frad/s roll=%+.2fdeg roll_rate=%+.3frad/s",
        pitch * 180.0 / kPi, pitch_rate, roll * 180.0 / kPi, roll_rate);
      return false;
    }

    const auto q = joint_positions(s);
    const FiveBarState left_state = kinematics_.forward(q[0], q[1]);
    const FiveBarState right_state = kinematics_.forward(q[2], q[3]);
    if (!left_state.valid || !right_state.valid) {
      RCLCPP_WARN(get_logger(), "Arm rejected: current five-bar state is invalid");
      return false;
    }
    if (recovery_enable_) {
      const double measured_height = 0.5 * (left_state.leg_length + right_state.leg_length);
      if (measured_height < recovery_height_min_m_ - 0.010 ||
        measured_height > recovery_height_max_m_ + 0.015)
      {
        RCLCPP_WARN(
          get_logger(), "Arm rejected: recovery height %.1fmm is outside safety bounds",
          1000.0 * measured_height);
        return false;
      }
      initialize_recovery(s, q, left_state, right_state, pitch_rate);
      armed_ = true;
      arm_transition_required_ = false;
      RCLCPP_WARN(
        get_logger(),
        "RECOVERY armed at pitch=%+.2fdeg height=%.1fmm; wheels and hips are now controlled",
        pitch * 180.0 / kPi, 1000.0 * measured_height);
      return true;
    }
    reset_height_controller(
      0.5 * (left_state.leg_length + right_state.leg_length), s.rc.left_y);
    IkSolution left_ik;
    IkSolution right_ik;
    if (!solve_target(
        q, height_target_m_, height_target_m_, left_ik, right_ik))
    {
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
    // Gravity-reference mode does not require switch 1: switch 3 captures only
    // the current wheel origin, never the current body attitude.
    balance_.calibrate_wheels(
      s.motors[kLeftWheel].position, s.motors[kRightWheel].position);
    balance_.reset();
    reset_roll_controller(roll, roll_rate);
    reset_policy_state();
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
      1000.0 * height_target_m_,
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
    recovery_active_ = false;
    normal_handoff_active_ = false;
    leg_ready_accumulated_s_ = 0.0;
    arm_transition_required_ = true;
    balance_.reset();
    reset_roll_controller(0.0, 0.0);
    reset_policy_state();
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
    const IkSolution & left_ik, const IkSolution & right_ik,
    const RollControlOutput & roll_output, const HeightControlOutput & height_output,
    const PolicyControlOutput & policy_output, double rc_right_x,
    const RecoveryControlOutput & recovery_output = RecoveryControlOutput{})
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
      left_ik.reconstruction_error_m, right_ik.reconstruction_error_m,
      // Roll values are appended so every pre-existing debug index remains unchanged.
      roll_output.roll_filtered_rad, roll_output.roll_rate_filtered_rad_s,
      roll_output.target_roll_rad, roll_output.target_roll_command_rad,
      roll_output.error_rad, roll_output.leg_difference_m,
      roll_output.left_target_y_m, roll_output.right_target_y_m, rc_right_x,
      // Height/policy values start at index 60. Policy observations 0..10 are
      // additionally copied verbatim to indices 75..85 for deployment auditing.
      height_output.rc_left_y, height_output.command_m, height_output.target_m,
      height_output.target_rate_mps, height_output.measured_m,
      height_output.measured_rate_mps, balance_debug.policy_wheel_position_m,
      balance_debug.policy_wheel_velocity_mps, policy_output.previous_action,
      policy_output.action, policy_output.residual_torque_nm,
      balance_debug.residual_torque_applied_each_nm,
      balance_debug.combined_common_torque_each_nm, policy_output.inference_time_us,
      policy_output.active ? 1.0 : 0.0,
      policy_output.observation[0], policy_output.observation[1],
      policy_output.observation[2], policy_output.observation[3],
      policy_output.observation[4], policy_output.observation[5],
      policy_output.observation[6], policy_output.observation[7],
      policy_output.observation[8], policy_output.observation[9],
      policy_output.observation[10],
      // Gravity-reference diagnostics start at index 86.
      gravity_up_body_.x, gravity_up_body_.y, gravity_up_body_.z,
      gravity_accel_norm_mps2_, attitude_reference_ready() ? 1.0 : 0.0,
      imu_reference_mode_ == "gravity" ? 1.0 : 0.0,
      imu_gravity_source_ == "orientation" ? 1.0 : 0.0,
      // Recovery diagnostics start at index 93. Observation 0..23 is copied
      // verbatim to indices 116..139 for deployment auditing.
      recovery_output.active ? 1.0 : 0.0,
      recovery_output.extension_unlocked ? 1.0 : 0.0,
      recovery_elapsed_s_, recovery_alignment_dwell_s_, recovery_output.success_dwell_s,
      recovery_output.measured_height_m, recovery_output.measured_height_rate_mps,
      recovery_output.height_target_m, recovery_output.height_target_rate_mps,
      recovery_output.measured_leg_angle_rad, recovery_output.leg_angle_target_rad,
      recovery_output.gravity_alignment_error_rad, recovery_output.wheel_position_m,
      recovery_output.wheel_velocity_mps, recovery_output.wheel_effort_nm,
      recovery_output.action[0], recovery_output.action[1], recovery_output.action[2],
      recovery_output.inference_time_us,
      recovery_output.joint_target[0], recovery_output.joint_target[1],
      recovery_output.joint_target[2], recovery_output.joint_target[3],
      recovery_output.observation[0], recovery_output.observation[1],
      recovery_output.observation[2], recovery_output.observation[3],
      recovery_output.observation[4], recovery_output.observation[5],
      recovery_output.observation[6], recovery_output.observation[7],
      recovery_output.observation[8], recovery_output.observation[9],
      recovery_output.observation[10], recovery_output.observation[11],
      recovery_output.observation[12], recovery_output.observation[13],
      recovery_output.observation[14], recovery_output.observation[15],
      recovery_output.observation[16], recovery_output.observation[17],
      recovery_output.observation[18], recovery_output.observation[19],
      recovery_output.observation[20], recovery_output.observation[21],
      recovery_output.observation[22], recovery_output.observation[23]};
    debug_pub_->publish(message);
  }

  void control_recovery(
    const Snapshot & s, double dt, double roll, double roll_rate,
    double pitch, double pitch_rate, double yaw_rate,
    const std::array<double, 4> & q, const std::array<double, 4> & qdot,
    const FiveBarState & left_state, const FiveBarState & right_state)
  {
    recovery_elapsed_s_ += dt;
    const double measured_height = 0.5 * (left_state.leg_length + right_state.leg_length);
    const double measured_leg_angle = recovery_mean_leg_angle(left_state, right_state);
    const double alignment_error = recovery_alignment_error(
      pitch, left_state, right_state);
    const double filtered_pitch_rate = recovery_pitch_rate_filter_.update(pitch_rate, dt);

    const double left_unwrapped = balance_config_.left_encoder_sign *
      recovery_left_wheel_unwrapper_.update(s.motors[kLeftWheel].position);
    const double right_unwrapped = balance_config_.right_encoder_sign *
      recovery_right_wheel_unwrapper_.update(s.motors[kRightWheel].position);
    const double wheel_position_m = balance_config_.wheel_radius_m *
      0.5 * (left_unwrapped + right_unwrapped);
    const double wheel_velocity_mps = balance_config_.wheel_radius_m * 0.5 * (
      balance_config_.left_encoder_sign * s.motors[kLeftWheel].velocity +
      balance_config_.right_encoder_sign * s.motors[kRightWheel].velocity);

    if (std::abs(pitch) > recovery_terminate_pitch_rad_) {
      disarm("recovery pitch failure bound exceeded");
      return;
    }
    if (std::abs(roll) > fall_cutoff_roll_rad_) {
      disarm("recovery roll safety angle exceeded");
      return;
    }
    if (measured_height < recovery_height_min_m_ - 0.010 ||
      measured_height > recovery_height_max_m_ + 0.015)
    {
      disarm("recovery virtual-leg height safety bound exceeded");
      return;
    }
    if (std::abs(wheel_position_m) > recovery_max_wheel_travel_m_) {
      RCLCPP_WARN(
        get_logger(),
        "Recovery wheel bound details: x=%+.3fm limit=%.3fm v=%+.3fm/s "
        "raw=(L=%+.3frad,R=%+.3frad) corrected=(L=%+.3frad,R=%+.3frad)",
        wheel_position_m, recovery_max_wheel_travel_m_, wheel_velocity_mps,
        s.motors[kLeftWheel].position, s.motors[kRightWheel].position,
        left_unwrapped, right_unwrapped);
      disarm("recovery wheel travel bound exceeded");
      return;
    }
    if (recovery_elapsed_s_ > recovery_max_duration_s_) {
      disarm("recovery timeout");
      return;
    }

    if (std::abs(alignment_error) <= recovery_alignment_unlock_rad_) {
      recovery_alignment_dwell_s_ += dt;
    } else {
      recovery_alignment_dwell_s_ = 0.0;
    }
    if (recovery_alignment_dwell_s_ >= recovery_alignment_unlock_dwell_s_) {
      recovery_extension_unlocked_ = true;
    }
    if (recovery_extension_unlocked_ &&
      std::abs(alignment_error) > recovery_alignment_relock_rad_)
    {
      recovery_extension_unlocked_ = false;
      recovery_alignment_dwell_s_ = 0.0;
    }

    recovery_policy_phase_s_ += dt;
    recovery_observation_elapsed_s_ += dt;
    if (recovery_policy_phase_s_ + 1.0e-9 >= recovery_policy_period_s_) {
      recovery_policy_phase_s_ = std::fmod(recovery_policy_phase_s_, recovery_policy_period_s_);
      const double observation_dt = std::max(recovery_observation_elapsed_s_, 1.0e-6);
      const double measured_height_rate =
        (measured_height - recovery_last_measured_height_m_) / observation_dt;
      recovery_last_measured_height_m_ = measured_height;
      recovery_observation_elapsed_s_ = 0.0;

      recovery_debug_.observation = make_recovery_observation(
        pitch, pitch_rate, wheel_position_m, wheel_velocity_mps,
        measured_height, measured_height_rate, measured_leg_angle,
        alignment_error, q, qdot);
      std::array<float, RecoveryPolicyRunner::kActionSize> raw_action{};
      std::string inference_error;
      const auto inference_start = std::chrono::steady_clock::now();
      const bool inference_ok = recovery_policy_.infer(
        recovery_debug_.observation, raw_action, inference_error);
      recovery_debug_.inference_time_us = 1.0e6 * std::chrono::duration<double>(
        std::chrono::steady_clock::now() - inference_start).count();
      if (!inference_ok) {
        disarm("recovery ONNX inference failed: " + inference_error);
        return;
      }
      for (std::size_t i = 0; i < raw_action.size(); ++i) {
        recovery_previous_action_[i] = clamp_value(
          static_cast<double>(raw_action[i]), -1.0, 1.0);
        recovery_debug_.action[i] = recovery_previous_action_[i];
      }

      recovery_requested_wheel_effort_nm_ = clamp_value(
        recovery_previous_action_[0] * recovery_wheel_torque_limit_nm_,
        -recovery_wheel_torque_limit_nm_, recovery_wheel_torque_limit_nm_);
      double requested_height_rate = clamp_value(
        recovery_height_rate_center_mps_ +
        recovery_height_rate_span_mps_ * recovery_previous_action_[1],
        recovery_height_rate_min_mps_, recovery_height_rate_max_mps_);
      if (!recovery_extension_unlocked_ ||
        std::abs(alignment_error) > recovery_alignment_relock_rad_)
      {
        requested_height_rate = 0.0;
      }
      const double old_height_target = recovery_height_target_m_;
      recovery_height_target_m_ = clamp_value(
        recovery_height_target_m_ + requested_height_rate * recovery_policy_period_s_,
        recovery_height_min_m_, recovery_height_max_m_);
      recovery_height_target_rate_mps_ =
        (recovery_height_target_m_ - old_height_target) / recovery_policy_period_s_;

      const double requested_leg_angle =
        recovery_previous_action_[2] * recovery_leg_angle_limit_rad_;
      const double max_leg_angle_step =
        recovery_leg_angle_rate_rad_s_ * recovery_policy_period_s_;
      recovery_leg_angle_target_rad_ += clamp_value(
        requested_leg_angle - recovery_leg_angle_target_rad_,
        -max_leg_angle_step, max_leg_angle_step);
      recovery_debug_.measured_height_rate_mps = measured_height_rate;
    }

    const double wheel_effort_step = recovery_wheel_torque_slew_nm_s_ * dt;
    recovery_wheel_effort_nm_ += clamp_value(
      recovery_requested_wheel_effort_nm_ - recovery_wheel_effort_nm_,
      -wheel_effort_step, wheel_effort_step);

    IkSolution left_ik;
    IkSolution right_ik;
    std::array<double, 4> q_des{};
    if (!solve_recovery_target(q, q_des, left_ik, right_ik)) {
      disarm("recovery virtual-leg IK is invalid");
      return;
    }
    recovery_joint_target_ = q_des;

    const bool command_enable = !dry_run_;
    const std::array<std::size_t, 4> joint_indices{
      kLeftJointA, kLeftJointB, kRightJointA, kRightJointB};
    double max_recovery_joint_error = 0.0;
    double max_recovery_joint_torque = 0.0;
    for (std::size_t i = 0; i < joint_indices.size(); ++i) {
      const double physical_joint_torque = clamp_value(
        recovery_joint_kp_ * (q_des[i] - q[i]) - recovery_joint_kd_ * qdot[i],
        -recovery_joint_torque_limit_nm_, recovery_joint_torque_limit_nm_);
      max_recovery_joint_error = std::max(
        max_recovery_joint_error, std::abs(q_des[i] - q[i]));
      max_recovery_joint_torque = std::max(
        max_recovery_joint_torque, std::abs(physical_joint_torque));
      const double motor_torque = calibrations_[joint_indices[i]].torque_sign *
        physical_joint_torque;
      motor_pubs_[joint_indices[i]]->publish(make_mit_command(
        command_enable && joint_control_enable_, 0.0, 0.0, 0.0, 0.0,
        motor_torque, recovery_joint_torque_limit_nm_));
    }
    motor_pubs_[kLeftWheel]->publish(make_mit_command(
      command_enable && balance_control_enable_, 0.0, 0.0, 0.0, 0.0,
      recovery_wheel_output_sign_ * balance_config_.left_encoder_sign *
      recovery_wheel_effort_nm_,
      recovery_wheel_torque_limit_nm_));
    motor_pubs_[kRightWheel]->publish(make_mit_command(
      command_enable && balance_control_enable_, 0.0, 0.0, 0.0, 0.0,
      recovery_wheel_output_sign_ * balance_config_.right_encoder_sign *
      recovery_wheel_effort_nm_,
      recovery_wheel_torque_limit_nm_));

    const bool stable =
      std::abs(pitch) <= recovery_success_pitch_rad_ &&
      std::abs(alignment_error) <= recovery_success_alignment_rad_ &&
      std::abs(pitch_rate) <= recovery_success_pitch_rate_rad_s_ &&
      std::abs(filtered_pitch_rate) <= recovery_success_pitch_rate_rad_s_ &&
      std::abs(measured_height - recovery_goal_height_m_) <=
      recovery_success_height_tolerance_m_ &&
      std::abs(wheel_velocity_mps) <= recovery_success_wheel_velocity_mps_;
    recovery_success_dwell_accumulated_s_ = stable ?
      recovery_success_dwell_accumulated_s_ + dt : 0.0;

    recovery_debug_.active = true;
    recovery_debug_.extension_unlocked = recovery_extension_unlocked_;
    recovery_debug_.joint_target = q_des;
    recovery_debug_.measured_height_m = measured_height;
    recovery_debug_.height_target_m = recovery_height_target_m_;
    recovery_debug_.height_target_rate_mps = recovery_height_target_rate_mps_;
    recovery_debug_.measured_leg_angle_rad = measured_leg_angle;
    recovery_debug_.leg_angle_target_rad = recovery_leg_angle_target_rad_;
    recovery_debug_.gravity_alignment_error_rad = alignment_error;
    recovery_debug_.wheel_position_m = wheel_position_m;
    recovery_debug_.wheel_velocity_mps = wheel_velocity_mps;
    recovery_debug_.wheel_effort_nm = recovery_wheel_effort_nm_;
    recovery_debug_.success_dwell_s = recovery_success_dwell_accumulated_s_;

    HeightControlOutput height_debug;
    height_debug.measured_m = measured_height;
    height_debug.measured_rate_mps = recovery_debug_.measured_height_rate_mps;
    height_debug.command_m = recovery_goal_height_m_;
    height_debug.target_m = recovery_height_target_m_;
    height_debug.target_rate_mps = recovery_height_target_rate_mps_;
    BalanceDebug balance_debug;
    balance_debug.policy_wheel_position_m = wheel_position_m;
    balance_debug.policy_wheel_velocity_mps = wheel_velocity_mps;
    balance_debug.combined_common_torque_each_nm = recovery_wheel_effort_nm_;
    RollControlOutput roll_debug;
    roll_debug.roll_filtered_rad = roll;
    roll_debug.roll_rate_filtered_rad_s = roll_rate;
    publish_debug(
      dt, pitch, pitch_rate, q, q_des, left_state, right_state,
      VmcOutput{}, VmcOutput{}, balance_debug, left_ik, right_ik,
      roll_debug, height_debug, PolicyControlOutput{}, s.rc.right_x, recovery_debug_);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 200,
      "state=RECOVERY pitch=%+.2fdeg align=%+.2fdeg height=%.1f/%.1fmm "
      "leg_angle=%+.1f/%+.1fdeg qerr=%.3frad tau_max=%.3fNm unlocked=%s "
      "rate=%+.2f/%+.2frad/s x=%+.3fm v=%+.3fm/s dwell=%.3fs "
      "action=(%+.3f,%+.3f,%+.3f) wheel=%+.3f/%+.3fNm",
      pitch * 180.0 / kPi, alignment_error * 180.0 / kPi,
      1000.0 * measured_height, 1000.0 * recovery_height_target_m_,
      measured_leg_angle * 180.0 / kPi,
      recovery_leg_angle_target_rad_ * 180.0 / kPi,
      max_recovery_joint_error,
      max_recovery_joint_torque,
      recovery_extension_unlocked_ ? "true" : "false",
      pitch_rate, filtered_pitch_rate, wheel_position_m, wheel_velocity_mps,
      recovery_success_dwell_accumulated_s_,
      recovery_previous_action_[0], recovery_previous_action_[1],
      recovery_previous_action_[2], recovery_wheel_effort_nm_,
      recovery_requested_wheel_effort_nm_);

    if (recovery_success_dwell_accumulated_s_ >= recovery_success_dwell_s_) {
      handoff_recovery_to_normal(
        s, pitch, pitch_rate, yaw_rate, measured_height, roll, roll_rate, dt);
    }
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
    if (s.imu.valid && age_seconds(steady_now, s.imu.received) <= imu_timeout_s_) {
      (void)update_gravity_reference(s.imu, dt);
    }
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

    double roll = 0.0;
    double roll_rate = 0.0;
    double pitch = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate = 0.0;
    if (!attitude(s.imu, roll, roll_rate, pitch, pitch_rate, yaw_rate)) {
      disarm("invalid attitude");
      return;
    }
    if (!recovery_active_ && std::abs(pitch) > fall_cutoff_rad_) {
      disarm("pitch fall angle exceeded");
      return;
    }
    if (std::abs(roll) > fall_cutoff_roll_rad_) {
      disarm("roll fall angle exceeded");
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

    if (recovery_active_) {
      control_recovery(
        s, dt, roll, roll_rate, pitch, pitch_rate, yaw_rate,
        q, qdot, left_state, right_state);
      return;
    }

    const HeightControlOutput height_output = update_height_controller(
      s.rc.left_y, left_state, right_state, dt);

    const RollControlOutput roll_output = update_roll_controller(
      roll, roll_rate, s.rc.right_x, height_output.target_m, dt,
      balance_armed_ && balance_control_enable_);

    IkSolution left_ik;
    IkSolution right_ik;
    if (!solve_target(
        q, roll_output.left_target_y_m, roll_output.right_target_y_m, left_ik, right_ik))
    {
      disarm("roll-adjusted five-bar target IK invalid");
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

    VmcConfig left_vmc_config = vmc_config_;
    VmcConfig right_vmc_config = vmc_config_;
    left_vmc_config.target_y_m = roll_output.left_target_y_m;
    right_vmc_config.target_y_m = roll_output.right_target_y_m;

    const VmcOutput left_vmc = calculate_vmc(
      left_state, pitch,
      vmc_config_.total_supported_mass_kg * vmc_config_.left_load_fraction,
      left_vmc_config);
    const VmcOutput right_vmc = calculate_vmc(
      right_state, pitch,
      vmc_config_.total_supported_mass_kg * vmc_config_.right_load_fraction,
      right_vmc_config);

    const bool command_enable = !dry_run_;
    const auto transition_blend = [this](double duration_s) {
        if (!normal_handoff_active_ || duration_s <= 0.0) {
          return 1.0;
        }
        const double linear = clamp_value(normal_handoff_elapsed_s_ / duration_s, 0.0, 1.0);
        // Cubic smoothstep has zero slope at both ends, avoiding a target-velocity or
        // torque-slope discontinuity when either part of the transition starts or ends.
        return linear * linear * (3.0 - 2.0 * linear);
      };
    const double wheel_handoff_blend = transition_blend(recovery_wheel_handoff_blend_s_);
    const double joint_handoff_blend = transition_blend(recovery_joint_handoff_blend_s_);
    std::array<double, 4> blended_q_des = q_des;
    if (normal_handoff_active_) {
      for (std::size_t i = 0; i < blended_q_des.size(); ++i) {
        blended_q_des[i] = normal_handoff_joint_target_[i] + joint_handoff_blend *
          (q_des[i] - normal_handoff_joint_target_[i]);
      }
    }
    const std::array<double, 4> p_des_motor{
      calibrations_[kLeftJointA].joint_to_motor(blended_q_des[0]),
      calibrations_[kLeftJointB].joint_to_motor(blended_q_des[1]),
      calibrations_[kRightJointA].joint_to_motor(blended_q_des[2]),
      calibrations_[kRightJointB].joint_to_motor(blended_q_des[3])};
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
        enabled ? joint_mit_kd_ : 0.0,
        enabled ? joint_handoff_blend * joint_torque[i] : 0.0,
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
        reset_policy_state();
        balance_.arm(make_balance_input(s, pitch, pitch_rate, yaw_rate, dt));
        balance_armed_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Leg target reached; wheel balance armed. max_error=%.4f rad max_velocity=%.4f rad/s",
          max_joint_error, max_joint_velocity);
      }
    }

    BalanceOutput balance_output;
    PolicyControlOutput policy_output;
    if (balance_control_enable_ && balance_armed_) {
      const BalanceInput balance_input = make_balance_input(s, pitch, pitch_rate, yaw_rate, dt);
      balance_output = balance_.prepare_update(balance_input);
      double residual_torque_nm = 0.0;
      double final_torque_limit_nm = balance_config_.torque_limit_each_nm;
      if (policy_enable_) {
        policy_output.active = true;
        policy_output.previous_action = policy_previous_action_;
        policy_output.observation = make_policy_observation(
          pitch, pitch_rate, height_output, balance_output.debug);
        float raw_action = 0.0F;
        std::string policy_error;
        const auto inference_start = std::chrono::steady_clock::now();
        const bool inference_ok = policy_.infer(
          policy_output.observation, raw_action, policy_error);
        policy_output.inference_time_us = 1.0e6 * std::chrono::duration<double>(
          std::chrono::steady_clock::now() - inference_start).count();
        if (!inference_ok) {
          disarm("ONNX policy inference failed: " + policy_error);
          return;
        }
        policy_output.action = clamp_value(static_cast<double>(raw_action), -1.0, 1.0);
        policy_output.residual_torque_nm =
          policy_output.action * policy_residual_scale_nm_;
        residual_torque_nm = policy_output.residual_torque_nm;
        final_torque_limit_nm = policy_final_torque_limit_each_nm_;
        policy_previous_action_ = policy_output.action;
      }
      balance_.finalize_update(
        balance_input, residual_torque_nm, final_torque_limit_nm, balance_output);
    }
    const bool wheel_enable = command_enable && balance_control_enable_ && balance_armed_;
    const double left_wheel_command_nm = normal_handoff_active_ ?
      (1.0 - wheel_handoff_blend) * recovery_wheel_output_sign_ *
      balance_config_.left_encoder_sign *
      normal_handoff_wheel_effort_nm_ +
      wheel_handoff_blend * balance_output.left_motor_torque_nm :
      balance_output.left_motor_torque_nm;
    const double right_wheel_command_nm = normal_handoff_active_ ?
      (1.0 - wheel_handoff_blend) * recovery_wheel_output_sign_ *
      balance_config_.right_encoder_sign *
      normal_handoff_wheel_effort_nm_ +
      wheel_handoff_blend * balance_output.right_motor_torque_nm :
      balance_output.right_motor_torque_nm;
    motor_pubs_[kLeftWheel]->publish(make_mit_command(
      wheel_enable, 0.0, 0.0, 0.0, 0.0,
      wheel_enable ? left_wheel_command_nm : 0.0,
      balance_config_.hard_torque_limit_each_nm));
    motor_pubs_[kRightWheel]->publish(make_mit_command(
      wheel_enable, 0.0, 0.0, 0.0, 0.0,
      wheel_enable ? right_wheel_command_nm : 0.0,
      balance_config_.hard_torque_limit_each_nm));

    if (normal_handoff_active_) {
      normal_handoff_elapsed_s_ += dt;
      if (normal_handoff_elapsed_s_ >= std::max(
          recovery_wheel_handoff_blend_s_, recovery_joint_handoff_blend_s_))
      {
        normal_handoff_active_ = false;
        RCLCPP_INFO(
          get_logger(), "NORMAL handoff complete (wheel=%.3fs, joint=%.3fs)",
          recovery_wheel_handoff_blend_s_, recovery_joint_handoff_blend_s_);
      }
    }

    publish_debug(
      dt, pitch, pitch_rate, q, q_des, left_state, right_state, left_vmc,
      right_vmc, balance_output.debug, left_ik, right_ik, roll_output,
      height_output, policy_output, s.rc.right_x);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 200,
      "state=%s pitch=%+.2fdeg roll=%+.2f/%+.2fdeg dLeg=%+.1fmm max_qerr=%.3frad "
      "height=%.1f/%.1fmm x=%.4fm v=%+.4fm/s B_L=(%.1f,%.1f)mm B_R=(%.1f,%.1f)mm "
      "policy=%+.3f residual=%+.3fNm handoff=%.2f/%.2f wheel_cmd=(%+.3f,%+.3f)Nm",
      balance_armed_ ? "BALANCE" : "LEG_POSITIONING",
      pitch * 180.0 / kPi, roll_output.roll_filtered_rad * 180.0 / kPi,
      roll_output.target_roll_rad * 180.0 / kPi,
      roll_output.leg_difference_m * 1000.0, max_joint_error,
      1000.0 * height_output.measured_m, 1000.0 * height_output.target_m,
      balance_output.debug.position_m, balance_output.debug.velocity_mps,
      left_state.x * 1000.0, left_state.y * 1000.0,
      right_state.x * 1000.0, right_state.y * 1000.0,
      policy_output.action, policy_output.residual_torque_nm,
      wheel_handoff_blend, joint_handoff_blend,
      left_wheel_command_nm, right_wheel_command_nm);
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
  PolicyRunner policy_{};
  RecoveryPolicyRunner recovery_policy_{};

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

  bool height_control_enable_{true};
  double height_center_m_{0.125};
  double height_min_m_{0.090};
  double height_max_m_{0.160};
  double height_target_slew_rate_mps_{0.060};
  double height_rc_deadband_{0.08};
  double height_rc_sign_{1.0};
  double height_command_m_{0.125};
  double height_target_m_{0.125};
  double height_target_rate_mps_{0.0};

  bool policy_enable_{true};
  std::string policy_model_path_{};
  double policy_residual_scale_nm_{0.060};
  double policy_final_torque_limit_each_nm_{0.260};
  double policy_previous_action_{0.0};

  bool recovery_enable_{true};
  std::string recovery_model_path_{};
  double recovery_policy_period_s_{0.010};
  double recovery_start_max_pitch_rad_{35.0 * kPi / 180.0};
  double recovery_start_max_pitch_rate_rad_s_{4.0};
  double recovery_terminate_pitch_rad_{70.0 * kPi / 180.0};
  double recovery_max_duration_s_{8.0};
  double recovery_max_wheel_travel_m_{0.80};
  double recovery_wheel_torque_limit_nm_{0.45};
  double recovery_wheel_output_sign_{1.0};
  double recovery_wheel_torque_slew_nm_s_{2.0};
  double recovery_height_min_m_{0.012};
  double recovery_height_max_m_{0.130};
  double recovery_goal_height_m_{0.120};
  double recovery_height_rate_center_mps_{0.040};
  double recovery_height_rate_span_mps_{0.100};
  double recovery_height_rate_min_mps_{-0.040};
  double recovery_height_rate_max_mps_{0.140};
  double recovery_leg_angle_limit_rad_{35.0 * kPi / 180.0};
  double recovery_leg_angle_rate_rad_s_{120.0 * kPi / 180.0};
  double recovery_tilt_unlock_min_height_m_{0.020};
  double recovery_tilt_unlock_full_height_m_{0.070};
  double recovery_alignment_unlock_rad_{5.0 * kPi / 180.0};
  double recovery_alignment_relock_rad_{8.0 * kPi / 180.0};
  double recovery_alignment_unlock_dwell_s_{0.10};
  double recovery_success_pitch_rad_{5.0 * kPi / 180.0};
  double recovery_success_alignment_rad_{5.0 * kPi / 180.0};
  double recovery_success_pitch_rate_rad_s_{0.35};
  double recovery_success_height_tolerance_m_{0.010};
  double recovery_success_wheel_velocity_mps_{0.30};
  double recovery_success_dwell_s_{0.15};
  double recovery_pitch_rate_filter_hz_{1.675};
  double recovery_joint_kp_{7.0};
  double recovery_joint_kd_{0.28};
  double recovery_joint_torque_limit_nm_{1.5};
  double recovery_left_alpha0_{2.4430524286};
  double recovery_left_beta0_{0.6977234730};
  double recovery_right_alpha0_{2.4438691429};
  double recovery_right_beta0_{0.6985402266};
  double recovery_hip_pivot_height_m_{0.1530};
  double recovery_chassis_com_height_m_{0.1895};
  double recovery_wheel_handoff_blend_s_{0.10};
  double recovery_joint_handoff_blend_s_{0.10};
  WrappedAngleUnwrapper recovery_left_wheel_unwrapper_{};
  WrappedAngleUnwrapper recovery_right_wheel_unwrapper_{};
  FirstOrderLowPass recovery_pitch_rate_filter_{};
  std::array<double, RecoveryPolicyRunner::kActionSize> recovery_previous_action_{};
  std::array<double, 4> recovery_joint_target_{};
  double recovery_height_target_m_{0.120};
  double recovery_height_target_rate_mps_{0.0};
  double recovery_leg_angle_target_rad_{0.0};
  double recovery_last_measured_height_m_{0.120};
  double recovery_wheel_effort_nm_{0.0};
  double recovery_requested_wheel_effort_nm_{0.0};
  double recovery_policy_phase_s_{0.0};
  double recovery_observation_elapsed_s_{0.0};
  double recovery_alignment_dwell_s_{0.0};
  double recovery_success_dwell_accumulated_s_{0.0};
  double recovery_elapsed_s_{0.0};
  bool recovery_extension_unlocked_{false};
  bool recovery_active_{false};
  bool normal_handoff_active_{false};
  double normal_handoff_elapsed_s_{0.0};
  double normal_handoff_wheel_effort_nm_{0.0};
  std::array<double, 4> normal_handoff_joint_target_{};
  RecoveryControlOutput recovery_debug_{};

  std::string imu_pitch_axis_{"pitch"};
  std::string imu_pitch_rate_axis_{"y"};
  std::string imu_roll_axis_{"roll"};
  std::string imu_roll_rate_axis_{"x"};
  std::string imu_yaw_rate_axis_{"z"};
  std::string imu_reference_mode_{"gravity"};
  std::string imu_gravity_source_{"orientation"};
  double imu_pitch_sign_{1.0};
  double imu_pitch_rate_sign_{1.0};
  double imu_roll_sign_{1.0};
  double imu_roll_rate_sign_{1.0};
  double imu_yaw_rate_sign_{1.0};
  double imu_mount_roll_rad_{0.0};
  double imu_mount_pitch_rad_{0.0};
  double imu_gravity_accel_sign_{1.0};
  double imu_gravity_filter_hz_{10.0};
  double imu_gravity_norm_min_mps2_{7.8};
  double imu_gravity_norm_max_mps2_{11.8};
  double imu_gravity_ready_dwell_s_{0.20};
  double imu_gravity_timeout_s_{0.20};
  FirstOrderLowPass gravity_x_filter_{};
  FirstOrderLowPass gravity_y_filter_{};
  FirstOrderLowPass gravity_z_filter_{};
  Vector3 gravity_up_body_{0.0, 0.0, 1.0};
  double gravity_accel_norm_mps2_{0.0};
  double gravity_ready_accumulated_s_{0.0};
  double gravity_invalid_elapsed_s_{0.0};
  bool gravity_filter_initialized_{false};
  bool gravity_reference_ready_{false};

  // Independent roll controller. It changes only left/right leg target Y and does not
  // alter the existing pitch, forward-velocity, or yaw wheel-torque controller.
  bool roll_control_enable_{false};
  double roll_trim_rad_{0.0};
  double roll_kp_leg_difference_m_per_rad_{0.080};
  double roll_kd_leg_difference_m_per_rad_s_{0.008};
  double roll_max_leg_difference_m_{0.016};
  double roll_leg_difference_slew_rate_mps_{0.040};
  double roll_max_target_rad_{5.0 * kPi / 180.0};
  double roll_target_slew_rate_rad_s_{20.0 * kPi / 180.0};
  double roll_rc_deadband_{0.08};
  double roll_rc_sign_{1.0};
  double roll_output_sign_{1.0};
  double roll_angle_filter_hz_{15.0};
  double roll_rate_filter_hz_{10.0};
  FirstOrderLowPass roll_filter_{};
  FirstOrderLowPass roll_rate_filter_{};
  double roll_target_command_rad_{0.0};
  double roll_leg_difference_m_{0.0};

  bool require_rc_{true};
  double imu_timeout_s_{0.05};
  double motor_timeout_s_{0.05};
  double rc_timeout_s_{0.20};
  double arm_max_tilt_rad_{10.0 * kPi / 180.0};
  double arm_max_pitch_rate_rad_s_{0.30};
  double fall_cutoff_rad_{25.0 * kPi / 180.0};
  double arm_max_roll_rad_{10.0 * kPi / 180.0};
  double arm_max_roll_rate_rad_s_{0.40};
  double fall_cutoff_roll_rad_{25.0 * kPi / 180.0};
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
