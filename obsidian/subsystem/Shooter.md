# Shooter and hood control

The shooter uses target velocity instead of fixed motor power, so it can correct for battery voltage changes and flywheel slowdown after a shot.

| Component | Role |
| --- | --- |
| Main motor encoder | Measures flywheel velocity |
| PIDFF controller | Corrects velocity error and predicts required power |
| Battery sensor | Scales feedforward for the current voltage |
| Follower motor | Copies the controlled motor output |
| Shooter velocity LUT | Linearly interpolates target velocity from goal distance |
| Hood LUT | Interpolates servo position from distance and measured flywheel velocity |

$$
power = PID(v_{target}-v_{measured}) +
{k_s\,sign(v_{target}) + k_v v_{target} \over V_{battery}}
$$

`MainTeleOp` updates both LUT targets every loop from the robot pose corrected by fixed-Limelight AprilTag relocalization. The velocity LUT replaces the previous two fixed speeds with a continuous distance-based target. The hood LUT uses measured velocity as well as corrected distance, so its angle can compensate when the flywheel is below or above its expected speed. The driver can apply a live hood trim during calibration.

The robot considers the shooter ready when measured velocity is within 70 ticks/s of the target. This state is shown through LEDs and telemetry. Feeding is still commanded by the driver; the current code does not automatically block a shot while the shooter is busy.

We tuned feedforward from logged voltage-versus-velocity data. `ke_kv_calculator.py` fits the relationship between applied voltage and flywheel speed so `k_v` can be chosen from measured behavior instead of guesswork.

## Measured result

| Test | Average absolute error | Within 70 ticks/s |
| --- | ---: | ---: |
| Closed loop, 2000 ticks/s | about 21.5 ticks/s | 94.9% |
| Open loop, 2000 ticks/s | about 168.2 ticks/s | 6.8% |
| Closed loop, 1300 ticks/s | about 22.2 ticks/s | 92.5% |

Closed-loop high-speed control reduced average velocity error by about 87% compared with open loop.

## Portfolio claim

> Goal distance sets a continuously interpolated flywheel target, encoder feedback and voltage-scaled PIDFF hold that speed, and a second LUT adjusts hood position using both distance and measured velocity.

## Evidence still needed

- A clear hood-LUT visualization or sample table.
- Scoreable field coverage before and after enabling all three LUTs.
- Physical spot-checks of representative simulator poses.
