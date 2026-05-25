# Turret-Mounted Limelight Relocalization
### FTC Team #25070 — DECODE Season (2025–2026)
#### Programming Notes by Coach

---

## What Is This?

We are adding a new feature to our robot this season that I want everyone on the programming side to understand, even if you're not the one writing it.

The idea is simple: **every time we shoot, we use the Limelight camera on the turret to read the AprilTag on the goal and correct the robot's believed position on the field.** This is called **relocalization**, and it is one of the most impactful things we can do to make our autonomous and late-game shooting reliable.

---

## The Problem We Are Solving

Our robot uses the **GoBilda Pinpoint** odometry computer to track where it is on the field. This works really well. But odometry is never perfect — it accumulates small errors over time from wheel slip, robot collisions, and tiny inconsistencies in the floor. After 30 seconds of play, those small errors add up to real centimeters of positional drift. After a collision with another robot, the error can jump unpredictably.

For a robot that just drives around, a few centimeters of error is fine. For a robot that is **shooting game elements into a goal from across the field**, a few centimeters can mean the difference between a score and a miss.

We need a way to **reset that drift periodically** using an absolute reference. That absolute reference is the **AprilTag on the goal**.

---

## The Opportunity We Have

Here is the key insight: **our Limelight is already aimed at the goal every single time we shoot.**

Our turret rotates to aim at the goal before firing. The Limelight is mounted on top of that turret. So at the exact moment we are about to shoot, the camera is already pointing directly at the goal AprilTag — Tag 20 for the Blue Goal, Tag 24 for the Red Goal. We get a perfect, clean line of sight to the very AprilTag we need for localization.

We do not need to add any extra robot movement. The relocalization happens naturally as part of the shooting sequence.

---

## The New Subsystem

We are building what I am calling the **Vision Relocalization Subsystem**. Think of it as a small, focused module that sits alongside our existing localization stack and has one job:

> *"When the turret is locked on target, read the AprilTag, compute where the robot actually is on the field, and push that corrected position back into Pinpoint."*

This subsystem will run during both TeleOp and Autonomous. In TeleOp the driver may not even notice it — every time the gunner shoots, the robot quietly becomes more confident about its position. In Autonomous it gives us a mid-match correction point we have never had before.

---

## The Tricky Part: The Turret Rotates

Here is the engineering challenge that makes this non-trivial.

Normally, when you configure a Limelight for robot localization, you tell it once where the camera sits relative to the center of the robot — how far forward, how far to the side, how high up. The Limelight uses that fixed offset to compute where the **robot** is, not just where the **camera** is.

But our camera is on a turret. The camera's position relative to the robot center **changes every time the turret rotates.** At 0 degrees, the camera is directly in front of the robot. At 90 degrees, it is offset to the right. The Limelight has no idea the turret even exists — it only knows what it sees through the lens.

So we have to compensate for this in software. Every loop cycle, the subsystem reads the current turret angle from the encoder and uses the known physical geometry of the turret arm to calculate exactly where the camera is sitting relative to the robot's center point at that moment. That offset is then used to reverse-calculate the true robot center position from the raw Limelight output.

Practically speaking: **if the Limelight says "the robot is here," we apply a small geometric correction based on the turret angle and tell the Pinpoint "actually, the robot is *here*."**

---

## MegaTag2 — The Algorithm We Use

The Limelight has two localization algorithms built in: MegaTag1 and MegaTag2.

We use **MegaTag2**. The reason is that MegaTag2 fuses the Limelight's camera reading with our robot's current heading from the Pinpoint IMU. This makes it significantly more accurate, especially when only a single AprilTag is visible — which is our typical situation in DECODE since there are only two usable goal tags on the entire field.

Every loop, the subsystem feeds the robot's current heading from Pinpoint into the Limelight. The Limelight uses that to constrain its localization math and gives us a much cleaner XY position estimate in return.

---

## Which AprilTags We Trust

This is important for everyone to know: **we only use Tags 20 and 24 for localization.** These are the Blue Goal and Red Goal tags respectively.

DECODE also has AprilTags on the Obelisk (Tags 21–23). We do **not** use those for localization. The Obelisk is a moveable game element and its position cannot be trusted as a fixed reference point. Using it would actually make our position *worse*. The subsystem has a filter that explicitly ignores any pose estimate that involves an Obelisk tag.

---

## Quality Filtering — We Don't Trust Every Reading

Not every AprilTag detection is a good one. Lighting, distance, motion blur, and robot occlusion can all produce bad readings. If we blindly accepted every detection and slammed it into our position estimate, a single bad frame could teleport our robot's believed position by half a meter inside the code — causing catastrophic path following errors.

The subsystem runs a quality check on every detection before accepting it. It checks things like:

- Is the tag large enough in the frame? (Too small means we're too far away and the reading is noisy.)
- Is the detection too recent or too old? (Stale readings from a cached frame are rejected.)
- Does the computed position make sense given where we already think we are? (A sudden 50cm jump is almost certainly a bad reading, not a real position change.)

Only readings that pass all of these checks are accepted and pushed into the Pinpoint.

---

## How It Fits Into the Match Flow

```
Normal operation:
    Pinpoint tracks position continuously as the robot drives.

On shoot trigger (turret locked on goal):
    Limelight reads the goal AprilTag.
    Quality check passes? ✓
    Turret angle correction applied.
    Corrected position injected into Pinpoint.
    Pedro Pathing updated with new position.
    Robot fires.

Result:
    Every shot resets any accumulated drift.
    The robot knows exactly where it is each time it fires.
```

---

## What This Means for the Team

**For drivers:** You do not need to do anything differently. The correction is automatic and invisible. The robot just becomes more accurate over the course of a match the more it shoots.

**For programmers:** This is a subsystem that interacts with three other systems — the Pinpoint, the Limelight, and Pedro Pathing. When you're working on it, be careful about coordinate systems. The Limelight returns positions in meters using the FTC field coordinate system. The Pinpoint uses inches. Pedro Pathing has its own coordinate space. Every handoff between these systems needs an explicit conversion — do not assume the units match.

**For mechanical:** The subsystem depends on knowing the physical dimensions of the turret arm precisely — specifically, the distance from the turret pivot to the camera lens, and the distance from the robot's center to the turret pivot. If the mechanism changes, those constants in the code need to be updated. Please communicate any design changes.

---

## Why This Is Worth the Complexity

A robot that knows where it is on the field is a robot that can make decisions. With accurate, continuously-corrected localization we can:

- **Improve shot consistency** — the robot always knows the exact distance and angle to the goal.
- **Enable smarter autonomous** — mid-auto corrections mean we can run longer, more complex paths with confidence.
- **Recover from collisions** — if another robot hits us and knocks our odometry off, the next shot automatically corrects us back.

This is not a nice-to-have. For a shooting robot in DECODE, this is the difference between a robot that aims well in the first 30 seconds and one that stays accurate for the entire 2.5-minute match.

---

*Notes compiled for internal team use — DECODE season 2025–2026.*
*If you have questions about the implementation, reach out before touching the subsystem code.*