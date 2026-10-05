| Topic | Type | Publisher | Subscriber | Meaning | Unit / range |
|---|---|---|---|---|---|
| /throttle | Float32 | none yet (teleop / controller later) | kinematic_bicycle | gas/brake pedal | [-1, 1], unitless (−1 full brake, +1 full gas) |
| /steer | Float32 | none yet (teleop / controller later) | kinematic_bicycle | front wheel angle | rad, + = left |
| /state | Odometry | kinematic_bicycle | lap_analyzer | x, y, heading, speed | x, y: m · heading: quaternion · speed: m/s |
| /path | Path | path_gen | lap_analyzer, rviz2 | track centreline | m |
| /telemetry/cte | Float32 | lap_analyzer | none yet (plot tools) | distance from centreline | m |
| /telemetry/heading_err_deg | Float32 | lap_analyzer | none yet (plot tools) | angle between car and track direction | deg |
| /telemetry/speed | Float32 | lap_analyzer | none yet (plot tools) | car speed | m/s |
| /telemetry/lap_time | Float32 | lap_analyzer | none yet (plot tools) | time of current lap | s |
## Milestone 2 — Vehicle model

State: x, y (rear axle, m), θ (heading, rad), v (speed, m/s). Inputs: throttle u ∈ [−1, 1], steering δ (rad, + = left).

```
ẋ = v·cos θ
ẏ = v·sin θ
θ̇ = (v / L)·tan δ                         L = 1.25 m
v̇ = k_a·u − (c_drag·v² + c_roll·v)        k_a = 4.0, c_drag = 0.005, c_roll = 0.05
```

Forward Euler, dt = 0.1 s: `x ← x + ẋ·dt` for all four states, then
- heading wrap: `θ ← atan2(sin θ, cos θ)` keeps θ in [−π, π] so angle errors stay correct
- speed clamp: `v ← clip(v, 0, 25)`, braking cannot reverse the car

Check: at u = 0.5, v̇ = 0 when 2.0 = 0.005v² + 0.05v → top speed ≈ 15.6 m/s, matching the plot.
At steer 0.3 rad the car drives a circle of radius R = L / tan δ ≈ 4 m.

## Milestone 3 — Teleoperation bridge

`/cmd_vel` (Twist) → `/throttle`, `/steer`:
```
throttle = clip(linear.x / 5.0, −1, 1)
steer    = clip(angular.z / 1.0, −1, 1) · 0.611 rad (35°)
```
Watchdog: published at 10 Hz; if no command for 0.5 s, throttle and steer go to 0 (the car coasts instead of running away).

## Milestone 4 — Longitudinal PID (cruise control)

```
e = v_target − v
u = Kp·e + Ki·∫e dt + Kd·de/dt          Kp = 1.0, Ki = 0.2, Kd = 0.05
```
- Anti-windup: the integral is clamped to ±2, and the output is clipped to [−1, 1].
- In cruise mode, `linear.x` is the target speed and the PID sets the throttle.

Observations (target 5 m/s):
- Speed rises and holds 5 m/s. Steady throttle ≈ 0.1 > 0: the I term supplies the gas needed against drag. P alone would settle below the target.
- On release, the target becomes 0 and the PID brakes to a stop.
- After stopping, throttle sits at −0.4 = Ki × (−2): the integral is held at its anti-windup limit instead of growing without bound.
