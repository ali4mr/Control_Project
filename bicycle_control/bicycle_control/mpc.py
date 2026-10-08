"""
High-Level Lateral Steering Controller: Extended Kinematic Bicycle MPC.
Solves a constrained non-linear program over prediction horizon N using SciPy,
optimizing steering angle and longitudinal acceleration (mapped to throttle).
"""

import math
import numpy as np
from scipy.optimize import minimize


class KinematicBicycleMPC:
    """Nonlinear Model Predictive Control for an Extended Kinematic Bicycle Model.

    Optimizes future control sequences u = [delta_k, a_k] where steering angle delta_k
    and longitudinal acceleration a_k (mapped to throttle effort) are the control inputs,
    forward-simulating a 4-state extended kinematic bicycle model x = [x, y, theta, v]^T.
    """

    def __init__(self, wheelbase=1.25, dt=0.1, horizon=10,
                 max_steer_rad=math.radians(35.0), k_a=4.0,
                 max_accel=None, max_brake=None, c_drag=0.005, c_roll=0.05):
        self.L = wheelbase
        self.dt = dt
        self.N = horizon
        self.max_steer_rad = max_steer_rad
        self.k_a = float(max_accel if max_accel is not None else k_a)
        self.c_drag = c_drag
        self.c_roll = c_roll

        # Weights: heavily penalize lateral CTE, heading error, and steering rate
        self.w_lat = 30.0
        self.w_long = 1.0
        self.w_yaw = 1.0
        self.w_v = 1.0
        self.w_steer = 0.2
        self.w_dsteer = 6.0
        self.w_accel = 0.1

        self.last_u = np.zeros(2 * self.N)  # warm-start [delta_0, a_0, delta_1, a_1, ...]
        self.last_accel = 0.0
        self.delay_steps = 1

    def solve(self, x0, ref_trajectory, current_steer=0.0):
        """Solves MPC optimization problem over horizon N.

        x0: [x, y, yaw, v]
        ref_trajectory: list of length N containing [x_ref, y_ref, yaw_ref, v_ref]
        current_steer: actual current steering angle in radians
        Returns: (steer_rad, throttle_cmd in [-1.0, 1.0])
        """
        N = min(self.N, len(ref_trajectory))
        if N < 2:
            return 0.0, 0.0

        bounds = []
        for _ in range(N):
            bounds.append((-self.max_steer_rad, self.max_steer_rad))
            bounds.append((-self.k_a, self.k_a))

        x_start, y_start, yaw_start, v_start = [float(s) for s in x0]
        for _ in range(self.delay_steps):
            x_start += v_start * math.cos(yaw_start) * self.dt
            y_start += v_start * math.sin(yaw_start) * self.dt
            yaw_start += (v_start / self.L) * math.tan(current_steer) * self.dt
            v_start += (self.last_accel - self.c_drag * v_start ** 2 - self.c_roll * v_start) * self.dt
            v_start = max(v_start, 0.0)
        ref_trajectory = list(ref_trajectory[self.delay_steps:]) + [ref_trajectory[-1]] * self.delay_steps

        def objective(u):
            x, y, yaw, v = x_start, y_start, yaw_start, v_start
            prev_delta = current_steer
            cost = 0.0
            for k in range(N):
                delta = u[2 * k]
                a = u[2 * k + 1]

                x += v * math.cos(yaw) * self.dt
                y += v * math.sin(yaw) * self.dt
                yaw += (v / self.L) * math.tan(delta) * self.dt
                v += (a - self.c_drag * v * v - self.c_roll * v) * self.dt
                v = max(v, 0.0)

                x_ref, y_ref, yaw_ref, v_ref = ref_trajectory[k]
                dx = x - x_ref
                dy = y - y_ref
                e_long = math.cos(yaw_ref) * dx + math.sin(yaw_ref) * dy
                e_lat = -math.sin(yaw_ref) * dx + math.cos(yaw_ref) * dy
                e_yaw = math.atan2(math.sin(yaw - yaw_ref), math.cos(yaw - yaw_ref))

                cost += self.w_lat * e_lat ** 2
                cost += self.w_long * e_long ** 2
                cost += self.w_yaw * e_yaw ** 2
                cost += self.w_v * (v - v_ref) ** 2
                cost += self.w_steer * delta ** 2
                cost += self.w_dsteer * (delta - prev_delta) ** 2
                cost += self.w_accel * a ** 2
                prev_delta = delta
            return cost

        if len(self.last_u) != 2 * N:
            self.last_u = np.zeros(2 * N)
        u_init = np.concatenate([self.last_u[2:], self.last_u[-2:]])

        result = minimize(objective, u_init, bounds=bounds, method='SLSQP',
                          options={'maxiter': 25, 'ftol': 1e-3})
        self.last_u = result.x

        delta_cmd = float(np.clip(result.x[0], -self.max_steer_rad, self.max_steer_rad))
        accel_cmd = float(result.x[1])
        self.last_accel = accel_cmd
        throttle_cmd = float(np.clip(accel_cmd / self.k_a, -1.0, 1.0))
        return delta_cmd, throttle_cmd
