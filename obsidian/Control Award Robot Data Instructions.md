# Control Award Robot Data Instructions

Execute in order. Do not freestyle.

## Before touching robot

- Charge 2 robot batteries.
- Open Logcat / driver station logging.
- Have a spreadsheet ready.
- Record: date, battery voltage, code branch, OpMode, alliance, field setup.
- Video every test from a fixed angle.
- Save every log with the test name.

## Output you must leave with

Fill these numbers:

| System     | Required result                                                           |
| ---------- | ------------------------------------------------------------------------- |
| Shooter    | spin-up time, average velocity error, max error, recovery time after shot |
| Turret     | settle time, final angle error, overshoot, peak current                   |
| LUT aiming | shots made before LUT, shots made after LUT, per field position           |
| Transfer   | detection success, false positives, false negatives, stop response time   |
| Telemetry  | loop Hz in competition mode, loop Hz in debug mode                        |
| Vision     | accepted corrections, rejected corrections, pose error before/after       |

If a number is missing, run the test again.

## Test 1 - Shooter velocity

Use `TestThroughput` or shooter debug OpMode.

1. Set target velocity to normal match value.
2. Start shooter from stopped.
3. Log target velocity, measured velocity, power, time, current.
4. Record time until velocity is within shooting tolerance.
5. Fire 10 artifacts, one at a time.
6. For each shot, record velocity before shot, minimum velocity after shot, recovery time.
7. Repeat for 2 battery voltages if possible: fresh and used.

Result table:

| Trial | Target vel | Spin-up s | Vel before | Min vel | Recovery s | Made shot? |
| --- | --- | --- | --- | --- | --- | --- |

## Test 2 - Turret motion profile

Use `KaTestOpMode`.

1. Command turret to fixed angles: `-60`, `-30`, `0`, `30`, `60` degrees.
2. Run 3 trials per angle.
3. Log target position, measured position, xref, vref, aref, velocity, current, time.
4. Record time until turret is within 1 degree.
5. Record final error, overshoot, peak current.
6. Watch for gear skip. If it skips, write the exact angle and current.

Result table:

| Target deg | Trial | Settle s | Final error deg | Overshoot deg | Peak current |
| --- | --- | --- | --- | --- | --- |

## Test 3 - Shot accuracy before/after LUT

This is the most important test.

1. Pick 6 field positions.
2. Mark them with tape.
3. Disable LUT.
4. Shoot 5 artifacts from each position.
5. Record made/missed.
6. Enable LUT.
7. Shoot 5 artifacts from the same positions.
8. Record made/missed.
9. For each position, record robot x, robot y, heading, turret target, turret measured, shooter velocity, hood angle.

Result table:

| Position | X | Y | Heading | No LUT made/5 | LUT made/5 | Notes |
| --- | --- | --- | --- | --- | --- | --- |

Stop condition:

- If LUT is worse at a position, keep that data.
- Adjust LUT only after finishing the full before/after set.

## Test 4 - Transfer automation

Use normal intake/transfer code.

1. Run 20 collection trials.
2. In each trial, feed artifacts normally.
3. Record whether each sensor detects correctly.
4. Record whether intake stops when 3 artifacts are collected.
5. Record false stop, missed stop, jam, current alert.
6. Record response time from third artifact detection to intake stop.

Result table:

| Trial | Detected 3? | Stopped? | Response ms | False positive? | False negative? | Jam/current? |
| --- | --- | --- | --- | --- | --- | --- |

## Test 5 - Telemetry cost

1. Run normal TeleOp in competition telemetry mode for 60 seconds.
2. Record average loop Hz and lowest loop Hz.
3. Run debug/trace telemetry for 60 seconds.
4. Record average loop Hz and lowest loop Hz.
5. Screenshot both telemetry modes.

Result table:

| Mode | Avg loop Hz | Min loop Hz | Notes |
| --- | --- | --- | --- |

## Test 6 - Vision relocalization

Only run if Limelight relocalization is connected.

1. Put robot at 5 known taped positions.
2. Record Pinpoint pose before vision correction.
3. Run vision correction.
4. Record accepted/rejected correction.
5. Record pose after correction.
6. Measure error from taped position before and after.

Result table:

| Position | Pinpoint error before | Accepted? | Error after | Rejected reason |
| --- | --- | --- | --- | --- |

## Final command

Before leaving, create one summary:

```text
Shooter improved/failed because: ___
Turret improved/failed because: ___
LUT improved/failed because: ___
Transfer improved/failed because: ___
Telemetry proved: ___
Vision status: competition / prototype / failed
```

No adjectives without numbers. No "it worked". Use the tables.
