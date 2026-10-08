"""
High-Level Controller: Model Predictive Path Integral (MPPI).
Sampling-based alternative to the gradient-based MPC, following the same idea as Nav2's MPPI controller:
sample many noisy control sequences, roll each out through the bicycle model, weight them by cost, average.
"""

import math
import numpy as np


class MPPIController:
    """Sampling-based MPC on the extended kinematic bicycle model."""

    def __init__(self, wheelbase=1.25, dt=0.1, horizon=15, num_samples=1000,
                 max_steer_rad=math.radians(35.0), k_a=4.0, c_drag=0.005, c_roll=0.05,
                 sigma_steer=0.12, sigma_accel=1.0, temperature=3.0,
                 track_half_width=1.0, seed=0):
        self.L = wheelbase
        self.dt = dt
        self.N = horizon
        self.K = num_samples
        self.max_steer_rad = max_steer_rad
        self.k_a = k_a
        self.c_drag = c_drag
        self.c_roll = c_roll
        self.sigma = np.array([sigma_steer, sigma_accel])
        self.lam = temperature
        self.track_half_width = track_half_width

        self.w_lat = 30.0
        self.w_yaw = 1.0
        self.w_v = 1.0
        self.w_dsteer = 6.0
        self.w_accel = 0.1
        self.w_off_track = 1000.0

        self.U = np.zeros((self.N, 2))
        self.last_accel = 0.0
        self.delay_steps = 1
        self.rng = np.random.default_rng(seed)
        self.last_solve_samples = None

    def _step(self, x, y, yaw, v, delta, a):
        x = x + v * np.cos(yaw) * self.dt
        y = y + v * np.sin(yaw) * self.dt
        yaw = yaw + (v / self.L) * np.tan(delta) * self.dt
        v = v + (a - self.c_drag * v * v - self.c_roll * v) * self.dt
        return x, y, yaw, np.maximum(v, 0.0)

    def solve(self, x0, ref_trajectory, current_steer=0.0):
        """Returns (steer_rad, throttle_cmd) from one MPPI iteration."""
        if len(ref_trajectory) < 2:
            return 0.0, 0.0

        x, y, yaw, v = [float(s) for s in x0]
        for _ in range(self.delay_steps):
            x, y, yaw, v = self._step(x, y, yaw, v, current_steer, self.last_accel)
        ref = list(ref_trajectory[self.delay_steps:]) + [ref_trajectory[-1]] * self.delay_steps
        ref = np.array(ref[:self.N] + [ref[-1]] * max(0, self.N - len(ref)))

        self.U = np.vstack([self.U[1:], self.U[-1:]])

        noise = self.rng.normal(size=(self.K, self.N, 2)) * self.sigma
        V = self.U[None, :, :] + noise
        V[:, :, 0] = np.clip(V[:, :, 0], -self.max_steer_rad, self.max_steer_rad)
        V[:, :, 1] = np.clip(V[:, :, 1], -self.k_a, self.k_a)

        xs = np.full(self.K, x)
        ys = np.full(self.K, y)
        yaws = np.full(self.K, yaw)
        vs = np.full(self.K, v)
        prev_delta = np.full(self.K, current_steer)
        cost = np.zeros(self.K)

        for k in range(self.N):
            delta = V[:, k, 0]
            a = V[:, k, 1]
            xs, ys, yaws, vs = self._step(xs, ys, yaws, vs, delta, a)

            x_ref, y_ref, yaw_ref, v_ref = ref[k]
            dx = xs - x_ref
            dy = ys - y_ref
            e_lat = -math.sin(yaw_ref) * dx + math.cos(yaw_ref) * dy
            e_yaw = np.arctan2(np.sin(yaws - yaw_ref), np.cos(yaws - yaw_ref))

            cost += self.w_lat * e_lat ** 2
            cost += self.w_yaw * e_yaw ** 2
            cost += self.w_v * (vs - v_ref) ** 2
            cost += self.w_dsteer * (delta - prev_delta) ** 2
            cost += self.w_accel * a ** 2
            cost += self.w_off_track * (np.abs(e_lat) > self.track_half_width)
            prev_delta = delta

        beta = cost.min()
        weights = np.exp(-(cost - beta) / self.lam)
        weights /= weights.sum()
        self.U = np.einsum('k,knm->nm', weights, V)
        self.last_solve_samples = (cost.min(), cost.mean())

        delta_cmd = float(np.clip(self.U[0, 0], -self.max_steer_rad, self.max_steer_rad))
        accel_cmd = float(self.U[0, 1])
        self.last_accel = accel_cmd
        throttle_cmd = float(np.clip(accel_cmd / self.k_a, -1.0, 1.0))
        return delta_cmd, throttle_cmd
