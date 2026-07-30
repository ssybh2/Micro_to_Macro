#!/usr/bin/env python3
"""Generate a continuous-time four-state LQR gain for the equivalent wheel inverted pendulum.

State order used by the controller:
    [pitch, pitch_rate, position_error, velocity_error]

The script first constructs the conventional model in order [x, xdot, pitch, pitch_rate],
solves CARE, then reorders K for the controller. The model is a useful starting point,
not a substitute for identification and real-robot verification.
"""

import argparse
import math
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--body-mass', type=float, required=True, help='Equivalent body mass m [kg]')
    parser.add_argument('--body-inertia', type=float, required=True, help='Body pitch inertia about COM I [kg m^2]')
    parser.add_argument('--pendulum-length', type=float, required=True, help='Wheel axle to body COM h [m]')
    parser.add_argument('--wheel-mass-each', type=float, required=True, help='One wheel+rotor mass [kg]')
    parser.add_argument('--wheel-inertia-each', type=float, default=0.0, help='One wheel rotational inertia [kg m^2]')
    parser.add_argument('--wheel-radius', type=float, required=True, help='Wheel radius [m]')
    parser.add_argument('--q-position', type=float, default=1.0)
    parser.add_argument('--q-velocity', type=float, default=1.0)
    parser.add_argument('--q-pitch', type=float, default=100.0)
    parser.add_argument('--q-pitch-rate', type=float, default=10.0)
    parser.add_argument('--r-torque', type=float, default=1.0)
    args = parser.parse_args()

    try:
        import numpy as np
        from scipy.linalg import solve_continuous_are
    except ImportError as exc:
        print('numpy and scipy are required: python3 -m pip install numpy scipy', file=sys.stderr)
        print(exc, file=sys.stderr)
        return 2

    m = args.body_mass
    I = args.body_inertia
    h = args.pendulum_length
    r = args.wheel_radius
    if min(m, h, r, args.wheel_mass_each) <= 0.0 or I < 0.0:
        raise ValueError('masses, length and radius must be positive; inertia must be non-negative')

    # Equivalent translating base mass includes wheel rotational inertia reflected through r.
    M = 2.0 * args.wheel_mass_each + 2.0 * args.wheel_inertia_each / (r * r)
    J = I + m * h * h
    ml = m * h
    denominator = (M + m) * J - ml * ml
    if denominator <= 0.0:
        raise ValueError('invalid model: mass matrix is not positive definite')

    g = 9.81
    # State z=[x, xdot, pitch, pitch_rate], input u=total wheel torque [N m].
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, -(m * m * h * h * g) / denominator, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, ((M + m) * m * g * h) / denominator, 0.0],
    ])
    B = np.array([
        [0.0],
        [J / (denominator * r)],
        [0.0],
        [-ml / (denominator * r)],
    ])
    Q = np.diag([
        args.q_position,
        args.q_velocity,
        args.q_pitch,
        args.q_pitch_rate,
    ])
    R = np.array([[args.r_torque]])
    P = solve_continuous_are(A, B, Q, R)
    K_z = np.linalg.solve(R, B.T @ P).reshape(-1)

    # u=-K_z[x,xdot,pitch,pitch_rate]. Controller YAML is ordered pitch,pitch_rate,x,xdot.
    K_controller = [K_z[2], K_z[3], K_z[0], K_z[1]]
    eig = np.linalg.eigvals(A - B @ K_z.reshape(1, -1))

    print(f'equivalent_base_mass_kg: {M:.10g}')
    print(f'pendulum_length_m: {h:.10g}')
    print('controller_state_order: [pitch, pitch_rate, position_error, velocity_error]')
    print('Use balance.mode: "lqr" and start with a small lqr.gain_scale.')
    print('lqr:')
    print('  gain_scale: 0.1')
    print(f'  k_pitch: {K_controller[0]:.12g}')
    print(f'  k_pitch_rate: {K_controller[1]:.12g}')
    print(f'  k_position: {K_controller[2]:.12g}')
    print(f'  k_velocity: {K_controller[3]:.12g}')
    print('closed_loop_eigenvalues:')
    for value in eig:
        print(f'  - {value.real:+.8g}{value.imag:+.8g}j')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
