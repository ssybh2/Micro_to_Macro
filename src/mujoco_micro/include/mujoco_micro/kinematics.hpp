#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace mujoco_micro
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct FiveBarGeometry
{
  double l1{0.0804};
  double l2{0.1200};
  double l3{0.1200};
  double l4{0.0804};
  double l5{0.0700};
  double singularity_epsilon{1.0e-5};
  double reachability_epsilon{1.0e-7};
};

struct FiveBarState
{
  bool valid{false};
  bool near_singularity{false};
  double alpha{0.0};
  double beta{0.0};
  double alpha_dot{0.0};
  double beta_dot{0.0};
  double x{0.0};
  double y{0.0};
  double x_dot{0.0};
  double y_dot{0.0};
  double leg_length{0.0};
  double leg_angle{0.0};
  double jacobian_det{0.0};
  // [xdot; ydot] = J [alpha_dot; beta_dot]
  double j11{0.0};
  double j12{0.0};
  double j21{0.0};
  double j22{0.0};
};

struct IkSolution
{
  bool valid{false};
  double alpha{0.0};
  double beta{0.0};
  double reconstruction_error_m{0.0};
};

class FiveBarKinematics
{
public:
  explicit FiveBarKinematics(const FiveBarGeometry & geometry = FiveBarGeometry{});

  void set_geometry(const FiveBarGeometry & geometry);
  const FiveBarGeometry & geometry() const noexcept {return geometry_;}

  FiveBarState forward(
    double alpha, double beta, double alpha_dot = 0.0, double beta_dot = 0.0) const;

  IkSolution inverse(
    double target_x, double target_y, double alpha_seed, double beta_seed,
    double reconstruction_tolerance_m = 2.0e-5) const;

private:
  struct CircleIntersections
  {
    std::array<Point2, 2> points{};
    std::size_t count{0};
  };

  CircleIntersections intersect_circles(
    const Point2 & c0, double r0, const Point2 & c1, double r1) const;

  static double wrap_to_pi(double angle);
  static double angular_distance(double a, double b);
  static bool finite(double value);

  FiveBarGeometry geometry_{};
};

}  // namespace mujoco_micro
