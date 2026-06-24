# 90-Second Control Award Pitch Script

This script is structured to be delivered in **exactly 90 seconds** (approx. 230 words at a normal speaking pace of 150 words/minute). It focuses on high-impact terminology, the engineering problem-solving process, and sets up strategic "hooks" (baits) to guide the judges' questions during Q&A.

---

## The Pitch Script

*(Start strong, speak with confidence)*

"Hi judges! For the Control Award, our software focus this season was **autonomous driver augmentation**—specifically, automating the aiming and shooting cycle to let our drivers focus entirely on positioning. 

To achieve this, we developed a field-centric turret. Instead of aiming at the physical goal, our turret targets a **virtual aim point** calculated in real-time using a **2D Position Lookup Table** and **Inverse Distance Weighting (IDW) interpolation**. To project this target when moving, we implemented **dynamic vector lead compensation**, which predicts the ring's flight time based on the robot's instantaneous velocity. We optimized this behavior using a custom Python simulation. *[Hook 1]*

For the shooter, consistency is key. We feed distance and real-time flywheel speed into a **2D Hood Angle LUT**. To combine inches and motor ticks without unit scale bias, our code runs a **normalized feature scaling** algorithm before interpolating. *[Hook 2]* 

Finally, our transfer system uses **debounced sensor fusion** of three distance sensors to automate intakes and stop jams. To maximize reliability, we implemented **sensorless shot detection** and **stall-detection turret homing** using motor current monitoring and low-pass filtered velocity derivatives, removing physical failure points from our robot. *[Hook 3]*

You can find our code snippets, math layouts, and testing graphs detailed on **pages 10, 11, and 12** of our portfolio. What questions do you have about our interpolation algorithms or simulation models?"

---

## Strategic Hooks Explained (How to Bait the Judges)

Your presentation should deliberately leave out "how" you did certain things so that the judges feel compelled to ask. Here are the three baits embedded in the script and what they prepare the judges to ask:

### 1. The Python Simulation & Virtual Aim Point Hook
* **The Pitch Line:** *"...our turret targets a virtual aim point... optimized using a custom Python simulation."*
* **The Bait:** Judges will want to know why you need a "virtual" target instead of the real goal, and how the Python simulation helped.
* **Your Prepared Answer:** Explain that structural asymmetry and spin cause shots to deflect depending on field position. The Python simulation modeled trajectories and let you map field coordinates $(X,Y)$ to offset targets.

### 2. The Normalized Feature Scaling Hook
* **The Pitch Line:** *"...To combine inches and motor ticks without unit scale bias, our code runs a normalized feature scaling algorithm..."*
* **The Bait:** Judges are technical; they will immediately be curious about "unit scale bias" and how you normalize these values.
* **Your Prepared Answer:** Show them [HoodAngleLut.java](file:///c:/Users/edurobots8/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/HoodAngleLut.java#L93-L111) and explain that a difference of 10 inches is massive for distance but 10 ticks/sec is negligible for velocity. Without normalization, Euclidean distance calculations in nearest-neighbor search would ignore velocity.

### 3. The Sensorless Automation Hook
* **The Pitch Line:** *"...we implemented sensorless shot detection and stall-detection turret homing using motor current monitoring..."*
* **The Bait:** Judges love mechanical simplification through smart software. They will ask how you detect shots or home the turret without sensors.
* **Your Prepared Answer:** Show how you track the flywheel's velocity derivative in [Shooter.java](file:///c:/Users/edurobots8/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Shooter.java#L129-L146) to detect the deceleration "spike" when a ring passes, and how you home the turret by monitoring current spikes when hitting hard stops in [Turret.java](file:///c:/Users/edurobots8/Documents/GitHub/FTC_DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/subsystem/Turret.java#L348-L363).
