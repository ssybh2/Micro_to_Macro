#include "mujoco_micro/kinematics.hpp"

#include <algorithm>
#include <limits>

namespace mujoco_micro
{
namespace
{
constexpr double kPi = 3.141592653589793238462643383279502884;
}

FiveBarKinematics::FiveBarKinematics(const FiveBarGeometry & geometry)
: geometry_(geometry)
{
}

void FiveBarKinematics::set_geometry(const FiveBarGeometry & geometry)
{
  geometry_ = geometry;
}

bool FiveBarKinematics::finite(const double value)
{
  return std::isfinite(value);
}

double FiveBarKinematics::wrap_to_pi(double angle)
{
  while (angle > kPi) {angle -= 2.0 * kPi;}
  while (angle < -kPi) {angle += 2.0 * kPi;}
  return angle;
}

double FiveBarKinematics::angular_distance(const double a, const double b)
{
  return std::abs(wrap_to_pi(a - b));
}

FiveBarKinematics::CircleIntersections FiveBarKinematics::intersect_circles(
  const Point2 & c0, const double r0, const Point2 & c1, const double r1) const
{
  CircleIntersections result;
  const double dx = c1.x - c0.x;
  const double dy = c1.y - c0.y;
  const double d = std::hypot(dx, dy);
  const double eps = std::max(geometry_.reachability_epsilon, 1.0e-12);

  if (!finite(d) || d < eps || d > r0 + r1 + eps || d < std::abs(r0 - r1) - eps) {
    return result;
  }

  const double a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d);
  double h2 = r0 * r0 - a * a;
  if (h2 < -eps) {
    return result;
  }
  h2 = std::max(0.0, h2);
  const double h = std::sqrt(h2);
  const double ux = dx / d;
  const double uy = dy / d;
  const Point2 midpoint{c0.x + a * ux, c0.y + a * uy};

  result.points[0] = Point2{midpoint.x - h * uy, midpoint.y + h * ux};
  if (h <= eps) {
    result.count = 1;
  } else {
    result.points[1] = Point2{midpoint.x + h * uy, midpoint.y - h * ux};
    result.count = 2;
  }
  return result;
}

FiveBarState FiveBarKinematics::forward(
  const double alpha, const double beta, const double alpha_dot, const double beta_dot) const
{
  FiveBarState out;
  out.alpha = alpha;
  out.beta = beta;
  out.alpha_dot = alpha_dot;
  out.beta_dot = beta_dot;

  const auto & g = geometry_;
  if (!finite(alpha) || !finite(beta) || !finite(alpha_dot) || !finite(beta_dot) ||
    !(g.l1 > 0.0) || !(g.l2 > 0.0) || !(g.l3 > 0.0) || !(g.l4 > 0.0) ||
    !(g.l5 > 0.0))
  {
    return out;
  }

  const Point2 a{g.l1 * std::cos(alpha), g.l1 * std::sin(alpha)};
  const Point2 c{g.l5 + g.l4 * std::cos(beta), g.l4 * std::sin(beta)};
  const auto intersections = intersect_circles(a, g.l2, c, g.l3);
  if (intersections.count == 0) {
    return out;
  }

  // Keep the same assembly branch as the generated VMC code: select the point
  // with the larger y coordinate (wheel centre below the two base pivots in the
  // user's coordinate convention, where +y points downward).
  Point2 b = intersections.points[0];
  if (intersections.count == 2 && intersections.points[1].y > b.y) {
    b = intersections.points[1];
  }

  const double phi_a = std::atan2(b.y - a.y, b.x - a.x);
  const double phi_c = std::atan2(b.y - c.y, b.x - c.x);
  const double denominator = std::sin(phi_a - phi_c);
  if (!finite(denominator)) {
    return out;
  }

  out.near_singularity = std::abs(denominator) < g.singularity_epsilon;
  if (out.near_singularity) {
    return out;
  }

  const double sa = std::sin(alpha - phi_a);
  const double sb = std::sin(phi_c - beta);
  out.j11 = g.l1 * std::sin(phi_c) * sa / denominator;
  out.j12 = g.l4 * std::sin(phi_a) * sb / denominator;
  out.j21 = -g.l1 * std::cos(phi_c) * sa / denominator;
  out.j22 = -g.l4 * std::cos(phi_a) * sb / denominator;
  out.jacobian_det = out.j11 * out.j22 - out.j12 * out.j21;

  if (!finite(out.j11) || !finite(out.j12) || !finite(out.j21) || !finite(out.j22)) {
    return FiveBarState{};
  }

  out.x = b.x;
  out.y = b.y;
  out.x_dot = out.j11 * alpha_dot + out.j12 * beta_dot;
  out.y_dot = out.j21 * alpha_dot + out.j22 * beta_dot;
  const double relative_x = b.x - 0.5 * g.l5;
  out.leg_length = std::hypot(relative_x, b.y);
  out.leg_angle = std::atan2(relative_x, b.y);
  out.valid = finite(out.x_dot) && finite(out.y_dot) && finite(out.leg_length) &&
    finite(out.leg_angle);
  return out;
}

IkSolution FiveBarKinematics::inverse(
  const double target_x, const double target_y, const double alpha_seed,
  const double beta_seed, const double reconstruction_tolerance_m) const
{
  IkSolution best;
  if (!finite(target_x) || !finite(target_y) || !finite(alpha_seed) || !finite(beta_seed)) {
    return best;
  }

  const Point2 o{0.0, 0.0};
  const Point2 d{geometry_.l5, 0.0};
  const Point2 b{target_x, target_y};

  const auto a_points = intersect_circles(o, geometry_.l1, b, geometry_.l2);
  const auto c_points = intersect_circles(d, geometry_.l4, b, geometry_.l3);
  if (a_points.count == 0 || c_points.count == 0) {
    return best;
  }

  double best_cost = std::numeric_limits<double>::infinity();
  for (std::size_t ai = 0; ai < a_points.count; ++ai) {
    const double alpha = std::atan2(a_points.points[ai].y, a_points.points[ai].x);
    for (std::size_t ci = 0; ci < c_points.count; ++ci) {
      const double beta = std::atan2(
        c_points.points[ci].y, c_points.points[ci].x - geometry_.l5);
      const FiveBarState reconstructed = forward(alpha, beta);
      if (!reconstructed.valid) {
        continue;
      }
      const double error = std::hypot(reconstructed.x - target_x, reconstructed.y - target_y);
      if (error > reconstruction_tolerance_m) {
        continue;
      }
      const double cost = angular_distance(alpha, alpha_seed) + angular_distance(beta, beta_seed) +
        1000.0 * error;
      if (cost < best_cost) {
        best_cost = cost;
        best.valid = true;
        best.alpha = alpha;
        best.beta = beta;
        best.reconstruction_error_m = error;
      }
    }
  }
  return best;
}

}  // namespace mujoco_micro
