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

## Milestone 5.1 — Velocity profiler

Curvature κ = 1/R, computed in the controller as Δheading / Δdistance between neighbouring waypoints.
Lateral acceleration in a bend is a_lat = v²·κ. Limiting it to a_max = 5 m/s² gives:
```
v_target = min( √(a_max / |κ|), v_max )      (v_max on straights, where κ ≈ 0)
```
Example: R = 5 m → κ = 0.2 → v = √(5 / 0.2) = 5 m/s.

## Milestone 5.2 — Lateral PID

CTE > 0 = car left of path; heading error > 0 = car pointing left of the path direction.
```
δ = −(Kp·CTE + Ki·∫CTE dt + Kd·dCTE/dt) − K_yaw·e_ψ       Kp = 0.8, Ki = 0.02, Kd = 0.15, K_yaw = 0.5
```
The minus signs steer against the error: left of path → steer right (δ < 0), and vice versa.
Integral clamped to ±1 (anti-windup); output clipped to ±35°.

## Milestone 5.3 — Pure Pursuit

1. Adaptive look-ahead: `Ld = clip(0.25·v + 0.8, 0.8, 2.5)` m. Faster → look further → smoother; too short → zig-zag, too long → corner cutting.
2. Target: from the nearest path point, walk forward to the first point at least Ld from the car.
3. Rotate the target into the car frame:
   `x_c = cos ψ·dx + sin ψ·dy`, `y_c = −sin ψ·dx + cos ψ·dy`, `α = atan2(y_c, x_c)`
4. Arc law: the circle through the rear axle and the target has curvature 2·sin α / Ld, so
   `δ = atan(2·L·sin α / Ld)`
Observation: smooth, no jitter. It starts turning before the corner because it uses a preview of the path.
## Milestone 6 — Free exploration: 4-wheel Ackermann kinematics

The bicycle model lumps both front wheels into one virtual wheel at angle δ. On a real car, all wheels
turn about one centre, so the inner wheel follows a tighter circle and must steer more.
With R = L / tan δ, L = 1.25 m, W = 1.18 m:
```
tan δ_inner = L / (R − W/2)      tan δ_outer = L / (R + W/2)
→ δ_left  = atan(L·tanδ / (L − (W/2)·tanδ)),  δ_right = atan(L·tanδ / (L + (W/2)·tanδ))   (sign-safe form)
```
| δ (bicycle) | inner | outer | difference |
|---|---|---|---|
| 5° | 5.2° | 4.8° | 0.4° |
| 20° | 23.7° | 17.3° | 6.5° |
| 35° | 46.3° | 27.8° | 18.5° |

Finding: the original sim published the same δ to both front steering joints. I changed `publish_joint_states`
to publish the Ackermann left/right angles, so the 3D model is geometrically correct (see docs/ackermann.png).
The physics still uses the bicycle model, which is valid precisely because Ackermann geometry gives all wheels a shared turn centre.
In ros2_control, `ackermann_steering_controller` (steering_controllers_library) does this conversion from wheelbase and track widths.

2D vs 3D: our 4-equation sim is instant and deterministic but has no tyre slip, load transfer or collisions; Gazebo/MVSim add physics
and sensors at a much higher compute and tuning cost.
MPC vs MPPI: MPC optimises one plan with a gradient-based solver; MPPI samples thousands of random control sequences and averages
the best. That handles obstacles and non-smooth costs easily but needs far more compute (GPU or many cores).
