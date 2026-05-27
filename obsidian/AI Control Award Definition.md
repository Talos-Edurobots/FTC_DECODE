# AI note - What the FTC Control Award is

This note is an AI-written reference for understanding the FTC Control Award before reviewing or writing our portfolio. It is not final portfolio wording.

Source checked: FIRST Tech Challenge 2025-2026 DECODE Competition Manual, Section 6 Awards, Control Award.

Official source:

- https://ftc-resources.firstinspires.org/ftc/game/manual

## Simple definition

The Control Award recognizes a team that uses software, sensors, and mechanical control to make the robot perform better during gameplay.

It is not just a programming award.
It is not just an autonomous award.
It is not just "we wrote a lot of code."

It is about using feedback and control to solve real game problems.

Good examples:

- using odometry to aim or drive more accurately
- using vision to correct robot position or align to a target
- using encoders to control shooter velocity
- using sensors to detect game elements
- using state machines to coordinate mechanisms
- using control loops to make mechanisms smooth, fast, and repeatable
- using telemetry/data to tune and verify the system

## What the manual requires

To be Control Award eligible, the portfolio must include:

1. Hardware and/or software control components on the robot.
2. Which challenge each component or system is intended to solve.
3. How each component or system works.
4. One or more hardware/software solutions that improve robot functionality using external feedback and control.

The portfolio must summarize the software, sensors, and mechanical control.
It should not include copies of code.

## What "external feedback and control" means

External feedback means the robot is measuring something from the real world or from hardware, then using that measurement to change behavior.

Examples:

- encoder velocity used to adjust shooter motor power
- turret encoder used to correct turret angle
- odometry pose used to calculate turret aiming angle
- AprilTag vision used to correct position or aiming
- distance/color sensors used to detect artifacts
- current alerts used to stop a mechanism before a jam becomes worse

Non-examples:

- a timed motor command with no sensor feedback
- hardcoded servo positions with no measurement or decision logic
- driver manually correcting every action
- code organization by itself

Code architecture can support the award, but the award is won by gameplay-improving feedback systems.

## What judges are really looking for

Judges are trying to answer these questions:

1. What game problem did the team solve?
2. What sensors or feedback did the robot use?
3. What algorithm or control method used that feedback?
4. Did it work during matches or realistic testing?
5. Did the team understand why it worked?
6. Did the team learn from failures and iterate?
7. Is the system reliable enough to matter?

If the portfolio only says "we used PID," that is weak.

If the portfolio says "we used shooter encoder feedback with PIDFF and battery-voltage scaling to reduce velocity error from X to Y, which improved scoring consistency," that is strong.

## Required vs encouraged

### Required

These must be present:

- submitted portfolio
- control components listed
- challenge solved by each component
- explanation of how each works
- at least one feedback/control solution that improves robot functionality

If any required item is missing, the team is not really Control Award ready.

### Encouraged

These are not strictly required, but they separate average teams from winning teams:

- control solution works consistently during most matches
- reliability is discussed or demonstrated
- team explains how reliability could be improved
- engineering process is shown
- lessons learned are included

In practice, winning usually requires the encouraged parts too.

## What "portfolio-ready" means

A Control Award section is portfolio-ready when every major claim has:

```text
Problem -> Feedback -> Control method -> Result -> Evidence
```

Example:

```text
Problem:
The shooter missed when battery voltage dropped or after rapid shots.

Feedback:
Shooter motor encoder velocity and battery voltage.

Control method:
PIDFF velocity control with voltage-scaled feedforward.

Result:
The shooter waits until measured velocity is close to target before feeding.

Evidence:
Graph of target vs measured velocity and recovery time after each shot.
```

## Best format for the portfolio

Use a table first, then explain the strongest system in detail.

Suggested table:

| Control system | Game challenge | Feedback used | Control method | Evidence |
| --- | --- | --- | --- | --- |
| Shooter velocity | consistent shots | encoder velocity, battery voltage | PIDFF | velocity graph |
| Turret aiming | aim from field positions | odometry pose, turret encoder | geometry + profiled PIDFF | angle error / settle time |
| Transfer automation | avoid overfilling | distance sensors, current alert | state machine | detection trials |
| Telemetry | tune without slowing robot | loop timing, snapshots | modes + throttling | loop Hz data |
| Vision relocalization | odometry drift | AprilTag pose | filtered correction | accepted/rejected corrections |

After the table, pick one main innovation and explain it clearly.

For this robot, the likely main innovation is:

```text
Pose-based turret aiming + shooter velocity control + telemetry-driven tuning
```

If vision relocalization becomes integrated into match code, then it can become the main innovation.

## What counts as strong evidence

Strong evidence:

- before/after shot accuracy
- target vs measured velocity graph
- turret target vs measured angle graph
- average and max turret error
- shooter spin-up and recovery time
- sensor detection accuracy table
- loop frequency in competition vs debug telemetry
- match logs or repeated test results

Weak evidence:

- "it was accurate"
- "it worked better"
- "drivers liked it"
- "we tested it"
- screenshots with no numbers
- code snippets

The Control Award is much easier to win with numbers.

## Reliability expectations

The solution does not need to be perfect, but it should work consistently during most matches or most realistic tests.

Reliability can be shown by:

- repeated trials
- match logs
- fallback behavior
- filtering bad sensor readings
- limits and safety checks
- explaining known failure modes
- explaining what the robot does when feedback is missing

Examples:

- If Limelight result is invalid, robot falls back to odometry aiming.
- If turret target is outside mechanical limits, target is clipped.
- If current draw is expensive to read, it is throttled.
- If intake sensors detect full robot, intake stops automatically.

## How to talk about failures

Failures are good if they show engineering judgment.

Strong failure wording:

- "We removed shoot-on-the-move because testing showed it reduced scoring consistency."
- "We rejected camera-only aiming because the Limelight lost the tag outside its field of view."
- "We added motion profiling because direct position control caused mechanical stress and skipped teeth."
- "We added a position-based LUT because aiming at the geometric target center was not accurate from every field position."

Weak failure wording:

- "It did not work."
- "It was inconsistent."
- "The sensor was bad."
- "We changed it because we wanted to."

Explain cause, test, decision, and result.

## What not to do

Do not fill the portfolio with code.

Do not describe every class equally.

Do not claim a prototype is a match feature unless it is actually used in match code.

Do not use vague words without measurements:

- accurate
- consistent
- fast
- reliable
- smooth
- optimized

Use numbers or describe exactly how the claim was tested.

## Control Award checklist

Before submitting, check:

- [ ] Does the portfolio name the control components?
- [ ] Does each component have a game challenge?
- [ ] Does each component explain how it works?
- [ ] Is at least one system clearly based on external feedback?
- [ ] Are sensors named precisely?
- [ ] Are algorithms named and explained simply?
- [ ] Is there evidence from testing or matches?
- [ ] Are reliability protections explained?
- [ ] Are failures and lessons learned included?
- [ ] Are prototypes separated from competition-ready systems?
- [ ] Is there no copied code?

## How this applies to our robot

Our strongest Control Award candidates are:

### Shooter velocity control

Feedback:

- shooter encoder velocity
- battery voltage

Control:

- PIDFF velocity controller
- busy/ready detection
- velocity drop / impact detection

Evidence needed:

- velocity graph
- spin-up time
- recovery time after shots
- shot success data

### Turret aiming

Feedback:

- robot pose
- turret encoder

Control:

- field geometry
- mechanical angle limits
- trapezoidal motion profile
- PIDFF position control

Evidence needed:

- turret settle time
- final angle error
- shot accuracy from multiple poses

### Position aim LUT

Feedback/input:

- robot field position

Control:

- interpolate virtual aim point
- aim turret at corrected target point

Evidence needed:

- before/after shot accuracy by field position
- field map showing sample points
- explanation of why distance-only correction was not enough

### Transfer automation

Feedback:

- three distance/color sensors
- intake current alert

Control:

- transfer state machine
- automatic stop when full or jammed

Evidence needed:

- detection accuracy
- response time
- false positives / false negatives

### Telemetry architecture

Feedback:

- subsystem snapshots
- loop timing
- current samples

Control/support:

- competition/debug/trace modes
- cost classes
- throttled current reads

Evidence needed:

- loop Hz comparison
- screenshots or logs showing mode separation
- explanation that telemetry supports tuning without hurting match performance

## Final understanding

The Control Award is won by proving that software and sensors made the robot better at the game.

The winning structure is:

```text
We had a game problem.
We measured the robot or field.
We used that feedback in a control system.
The robot improved.
We tested it enough to trust it.
We learned from failures and changed the design.
```

If a portfolio does that clearly, it is Control Award ready.
