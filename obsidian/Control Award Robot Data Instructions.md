# Control Award evidence checklist

Record the date, code revision, battery voltage, OpMode, alliance, field setup, and simulator version for every test.

## 1. Shooter velocity

Use the existing acceleration and shot-drop data to report:

- average absolute velocity error
- percentage within the 70 ticks/s ready tolerance
- spin-up and recovery time where timestamps are available
- closed-loop versus open-loop result

## 2. LUT scoreable field coverage

This is the main LUT validation.

Before testing both alliances, verify that the shooter/hood distance helper uses the correct alliance goal. The current helper ignores its `isRed` argument.

1. Define a grid over every robot pose that is mechanically reachable and strategically useful.
2. Use one fixed heading rule and one scoring-success definition.
3. Run the old system: fixed goal target plus two fixed velocity/hood pairs.
4. Mark each grid cell as scoreable or not scoreable.
5. Run the current system with turret, shooter velocity, and hood LUTs enabled.
6. Mark the same cells and overlay the two maps.
7. Report scoreable cells, estimated scoreable area, and percentage increase.
8. Save failures and boundary cells.

| Configuration | Grid cells tested | Scoreable cells | Scoreable area | Coverage |
| --- | ---: | ---: | ---: | ---: |
| Before LUTs |  |  |  |  |
| After LUTs |  |  |  |  |

### Physical spot-check

Verify at least:

- one near pose
- one far pose
- one pose from each side of the goal
- two poses near the predicted coverage boundary
- one pose predicted not to score

The simulator establishes coverage; physical tests check that the model matches the robot.

## 3. Turret profile

Only needed for numerical motion claims. Log timestamp, target angle, measured angle, reference position/velocity/acceleration, power, and current.

| Target | Settle time | Final error | Overshoot | Peak current |
| ---: | ---: | ---: | ---: | ---: |
|  |  |  |  |  |

## 4. Limelight relocalization

Place the robot at taped field poses whose coordinates and headings are known. Introduce normal odometry drift by driving before each measurement.

1. Record the Pinpoint pose and error before vision correction.
2. Record tag ID, tag count, ambiguity/quality, and Limelight pose.
3. Record whether the observation was accepted or rejected.
4. If accepted, record corrected pose and remaining error.
5. If rejected, record the exact rejection reason.
6. Include trials with no visible tag and deliberately poor observations.

| Trial | Known pose | Pinpoint error before | Vision accepted | Error after | Rejection reason |
| --- | --- | ---: | --- | ---: | --- |
|  |  |  |  |  |  |

Report median and maximum pose error before and after accepted corrections, plus the number of accepted and rejected observations.

## 5. Transfer automation

Run 20 realistic collection trials. Record correct full detection, automatic stop, false positive, false negative, response time, and overcurrent events.

| Trial | Full detected | Stopped | Response ms | False stop | Missed stop | Overcurrent |
| --- | --- | --- | ---: | --- | --- | --- |
|  |  |  |  |  |  |  |

## 6. Telemetry cost

Run 60 seconds in competition mode and 60 seconds in debug mode.

| Mode | Average loop Hz | 1% low Hz | 0.1% low Hz |
| --- | ---: | ---: | ---: |
| Competition |  |  |  |
| Debug |  |  |  |

## Final evidence summary

```text
Shooter closed-loop improvement:
Scoreable coverage before and after LUTs:
Physical LUT spot-check result:
Limelight pose error before and after:
Accepted/rejected vision observations:
Turret profile result:
Transfer reliability:
Telemetry loop-rate result:
Rejected feature and lesson learned: shoot-on-the-move
```
