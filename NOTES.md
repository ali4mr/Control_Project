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
