# Intake and transfer automation

The transfer uses an intake motor, gate servo, three distance sensors, and intake-current protection. The fixed Limelight is mounted above the intake, outside the turret, so its robot-to-camera transform remains constant during aiming. A state machine keeps the mechanism behavior consistent.

| State | Intake | Gate | Purpose |
| --- | --- | --- | --- |
| `STOP` | Off | Closed | Hold artifacts and stop collection |
| `COLLECT` | On | Closed | Collect and store artifacts |
| `SHOOT` | On | Open | Feed artifacts into the shooter |

The three sensors update at 15 Hz. The robot is considered full after all three positions remain detected for 0.4 seconds. While collecting, either a full state or intake overcurrent automatically changes the transfer to `STOP`. The right LED indicates the stopped/full state.

The driver starts or stops collection and holds the shoot command to feed. The state machine automates the mechanism outputs and safety stop; it does not automatically wait for shooter readiness.

## Portfolio claim

> Three artifact sensors and motor-current feedback automatically stop collection when capacity is reached or a jam is detected, reducing driver observation and protecting the mechanism.

## Evidence still needed

- Collection trials with correct stops, false stops, and missed stops.
- Response time from confirmed full detection to motor stop.
- Jam/overcurrent test results.
