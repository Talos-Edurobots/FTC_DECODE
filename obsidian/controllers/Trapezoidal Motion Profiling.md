The trapezoidal motion profiling is an algorithm that generates velocity and position trajectories (xref, vref, aref) that the turret controller should follow. That specific algorithm is used to achieve fixed acceleration and splits at three different states (acceleration, constant velocity, deceleration), though the turret just accelerates and decelerates as shown on [[pos vs xref.png]] [[velocity vs vref.png]].  Then, a [[PID + FF]] controller takes as input the position and velocity errors and xref, vref, aref and outputs the expected motor power. That code is implemented and tested inside the [[motor package]]. 

$$
power=k_{a}*a_{ref} + k_{v}*v_{ref}+k_{s}*signum(v_{ref})+k_{p}*e_{p}(t)+k_{i}*\int_{0}^{t} e_{p}(t) \,dt-k_{d}*e_{v}(t)
$$
where $e_{p}(t)$ is position error and $e_{v}(t)$ is velocity error 


![](https://gm0.org/en/latest/_images/trapezoidal-motion-profiling-graph.png)

