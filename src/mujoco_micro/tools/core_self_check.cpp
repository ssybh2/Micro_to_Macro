#include <cmath>
#include <cstdlib>
#include <iostream>

#include "mujoco_micro/control_core.hpp"
#include "mujoco_micro/kinematics.hpp"

using mujoco_micro::FiveBarGeometry;
using mujoco_micro::FiveBarKinematics;
using mujoco_micro::JointCalibration;

int main()
{
  FiveBarGeometry geometry;
  FiveBarKinematics kinematics(geometry);

  const double alpha_seed = 2.9341493;
  const double beta_seed = 0.2074434;
  const auto ik = kinematics.inverse(0.035, 0.120, alpha_seed, beta_seed);
  if (!ik.valid) {
    std::cerr << "IK failed\n";
    return EXIT_FAILURE;
  }
  const auto fk = kinematics.forward(ik.alpha, ik.beta);
  if (!fk.valid) {
    std::cerr << "FK failed\n";
    return EXIT_FAILURE;
  }
  const double error = std::hypot(fk.x - 0.035, fk.y - 0.120);
  if (error > 2.0e-5) {
    std::cerr << "round-trip error too large: " << error << "\n";
    return EXIT_FAILURE;
  }

  JointCalibration left_a{-0.340847969, 3.50454569, -1.0, 1.0, -1.0};
  JointCalibration left_b{-0.629243851, 0.899913073, -1.0, 1.0, -1.0};
  const double p_a = left_a.joint_to_motor(ik.alpha);
  const double p_b = left_b.joint_to_motor(ik.beta);

  std::cout << "IK alpha=" << ik.alpha << " beta=" << ik.beta << "\n";
  std::cout << "FK x=" << fk.x << " y=" << fk.y << " error=" << error << "\n";
  std::cout << "left motor p_des A=" << p_a << " B=" << p_b << "\n";
  std::cout << "jacobian det=" << fk.jacobian_det << "\n";


  mujoco_micro::VmcConfig vmc_config;
  const auto vmc = mujoco_micro::calculate_vmc(fk, 0.0, 1.4, vmc_config);
  if (!std::isfinite(vmc.tau_a) || !std::isfinite(vmc.tau_b)) {
    std::cerr << "VMC output is non-finite\n";
    return EXIT_FAILURE;
  }

  mujoco_micro::BalanceController balance;
  mujoco_micro::BalanceConfig balance_config;
  balance_config.torque_limit_each_nm = 0.20;
  balance_config.output_gain_sign = -1.0;
  balance_config.left_motor_sign = -1.0;
  balance_config.right_motor_sign = 1.0;
  balance_config.left_encoder_sign = 1.0;
  balance_config.right_encoder_sign = -1.0;
  balance_config.cascade_attitude_k_pitch = 8.0;
  balance_config.cascade_attitude_k_pitch_rate = 0.06;
  balance_config.cascade_position_kp_rad_per_m = 0.12;
  balance_config.cascade_velocity_kd_rad_per_mps = 0.035;
  balance_config.cascade_position_to_pitch_sign = -1.0;
  balance.configure(balance_config);
  balance.calibrate_wheels(0.0, 0.0);
  mujoco_micro::BalanceInput balance_input;
  balance_input.pitch_rad = 2.0 * mujoco_micro::kPi / 180.0;
  balance_input.pitch_rate_rad_s = 0.0;
  balance_input.left_sequence = 1;
  balance_input.right_sequence = 1;
  balance.arm(balance_input);
  balance_input.left_sequence = 2;
  balance_input.right_sequence = 2;
  auto balance_output = balance.prepare_update(balance_input);
  const double baseline_common = balance_output.debug.common_torque_each_nm;
  balance.finalize_update(balance_input, 0.060, 0.260, balance_output);
  if (!std::isfinite(balance_output.left_motor_torque_nm) ||
    !std::isfinite(balance_output.right_motor_torque_nm))
  {
    std::cerr << "balance output is non-finite\n";
    return EXIT_FAILURE;
  }
  const double expected_common = mujoco_micro::clamp_value(
    baseline_common + 0.060, -0.260, 0.260);
  if (std::abs(balance_output.debug.combined_common_torque_each_nm - expected_common) > 1.0e-12 ||
    std::abs(
      balance_output.debug.residual_torque_applied_each_nm -
      (expected_common - baseline_common)) > 1.0e-12)
  {
    std::cerr << "residual common torque composition mismatch\n";
    return EXIT_FAILURE;
  }
  if (std::abs(balance_output.debug.policy_wheel_position_m) > 1.0e-12) {
    std::cerr << "policy wheel origin was not reset on balance arm\n";
    return EXIT_FAILURE;
  }
  std::cout << "VMC tau A=" << vmc.tau_a << " B=" << vmc.tau_b << "\n";
  std::cout << "balance wheel torque L=" << balance_output.left_motor_torque_nm
            << " R=" << balance_output.right_motor_torque_nm << "\n";

  // Expected motor targets for B=(35,120) mm using the retained real-robot calibration.
  if (std::abs(p_a - 0.69844872) > 2.0e-4 || std::abs(p_b + 0.40567443) > 2.0e-4) {
    std::cerr << "motor target mismatch\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
