This controller is used on the shooter, so it can keep a given velocity accurately. It is implemented and tested inside the [[motor package]]. The controller consists of 2 parts:
PID controller: the closed loop controller that reacts on velocity error

Feed forward: modeled open loop controller that eliminates static friction and predicts required voltage to keep a given velocity. This controller can be tuned automatically, by supplying different amounts of power and logging the flywheel's different velocities and fitting those points using regression. [[kv.png]]