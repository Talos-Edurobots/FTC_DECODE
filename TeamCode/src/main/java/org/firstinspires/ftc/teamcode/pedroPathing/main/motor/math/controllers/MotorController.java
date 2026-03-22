package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.LoopState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;

public interface MotorController {
    void init(MotionState currentState);
    double updateWithError(double error, MotionState motionState, LoopState loopState);
    double update(double target, MotionState motionState, LoopState loopState);
}
