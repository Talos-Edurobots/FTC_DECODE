package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

public interface MotorController {
    void update(double target, MotionState motionState, LoopState loopState);
    void update(double target, double current, LoopState loopState);
}
