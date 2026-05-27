## V1 PIDF based turret with limelight aiming
On out first iteration the turret had a limelight 3A on top of it that was scanning the april tag on the goal
### Pros
- The code implementation was easy
- The limelight doesn't create errors 
### Cons
- The turret's gears would skip
- The turret was slow so that there would not create mechanical stress and lock to the goal accurately 
- The limelight has a limited FOV, so the turret would lose track with the goal consistently
- The turret didn't aim correctly to all the field coordinates, because aiming at the center of the April tag doesn't necessarily mean correct aiming 

## V2 Trapezoidal Motion Profiling based turret with pinpoint aiming and shoot on the move
The Turret uses a [[Trapezoidal Motion Profiling]] to move to a desired angle. Using trigonometry we can calculate the angle in radians that the turret should be at any robot and target position. We set a goal position for each goal and the turret aims there using this equation:

$$
θ=atan2(x_{target}-x_{robot},y_{target}-y_{robot})-heading_{robot}
$$
Also, we implemented SOTM that we tested on the [[Simulator]], but we found out that it is inconsistent, we didn't use it and it made our cycles slower.
### Pros
- The turret is fast without mechanical stress
- The turret doesn't lose track of the goal, because at any time the pinpoint knows the robot's position
### Cons
- Motion profiling required more time and effort implementing, debugging and tuning
- The pinpoint drifts noticeably and the shots would not make it to the goal after a while
- It didn't solve the aiming problem from V1. The turret still aims to a single target position for every point of the field
- The limelight is not used
- Inconsistent SOTM 

## V3 Motion Profiling with sensor fusion and interpolated lookup table
The limelight is used again, but fixed under the turret. It is used to update the pinpoint's reading using a low pass lifter. That makes position readings much more accurate. Also the interpolated LUT analyzed on [[Turret LUT]] solves the aiming issues from previous iterations. We also decided to throw away SOTM because it didn't work for us

### Pros
- Consistent and accurate aiming
- Keeps the pros from the previous iteration
- The turret locks to the goal faster, because we removed the SOTM feature
### Cons
- sensor fusion can make noisy so it may be inconsistent
- more code complexity, more time for implementation needed
- No SOTM
