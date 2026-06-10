# Driver automations

| Driver action | Automated response |
| --- | --- |
| Drive | Field-centric acceleration-limited drive; stick click toggles slow mode |
| AprilTag visible | Fixed Limelight validates the observation and corrects accumulated Pinpoint pose drift |
| AprilTag unavailable | Pinpoint continues providing the pose without interrupting aiming |
| Collect | Transfer runs intake and closes the gate |
| Robot becomes full | Three sensors debounce the full state, stop intake, and update the LED |
| Intake jams | Overcurrent stops collection |
| Move anywhere in the scoring region | Corrected pose updates turret aim, shooter velocity, and hood position continuously |
| Aim | Turret follows a profiled target from the virtual aim-point LUT |
| Shoot | Holding one bumper opens the gate and feeds artifacts |

LEDs communicate shooter readiness and transfer state. Panels provides live pose, vision acceptance, correction size, LUT targets, and controller graphs during tuning.

The current code assists the driver but does not automatically prevent feeding when the shooter is below target velocity.
