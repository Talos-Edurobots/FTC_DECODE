## Shooter Velocity Control

The shooter solves the problem that flywheel shots change when the battery voltage drops or when an artifact slows the wheel down. Instead of giving the shooter a fixed power, we control it by target velocity.

| Part                       | What it does                                                                                    |
| -------------------------- | ----------------------------------------------------------------------------------------------- |
| Main shooter motor encoder | Measures real flywheel velocity                                                                 |
| [[PID + FF]] controller    | Corrects velocity error and predicts needed power                                               |
| Battery voltage sensor     | Scales feedforward so power is not tuned only for one battery voltage                           |
| Follower shooter motor     | Mirrors the controlled motor power                                                              |
| Hood servo                 | Changes shot angle for close/far shots                                                          |
| Telemetry snapshot         | Logs target velocity, measured velocity, filtered velocity, power, current, voltage, busy state |

The main shooter motor uses `VelocityControlledMotor`. Every loop it reads encoder velocity, compares it to `Shooter.targetVelocity`, and outputs motor power using PID feedback + feedforward:

$$
power = PID(v_{target}-v_{measured}) + {k_s sign(v_{target}) + k_v v_{target} \over V_{battery}}
$$

The PID part reacts to velocity error. The feedforward part predicts how much power the flywheel should need before the error happens. This matters because the shooter should not wait until it is already slow to start correcting.

The shooter has 2 motors. Only the main motor is closed-loop from encoder velocity. The second motor is a follower and copies the same power, so both wheels apply the same output while the controller uses one clean feedback source.

### Match behavior (Automation)
We have developed the 


The robot also calculates `shooter.isBusy()`:

```text
busy = abs(target velocity - measured velocity) > 70 ticks/s
```
