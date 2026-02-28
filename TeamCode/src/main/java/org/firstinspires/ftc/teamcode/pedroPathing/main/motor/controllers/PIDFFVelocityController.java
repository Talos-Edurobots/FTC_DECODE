package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.controllers;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.LoopState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;

class PIDFFVelocityController implements MotorController{
    double lastError, integral;
    PIDFFCoefficients coef;

    public PIDFFVelocityController(PIDFFCoefficients coefficients) {
        this.coef = coefficients;
    }

    @Override
    public void init(MotionState currentState) {

    }

    @Override
    public double update(double target, MotionState motionState, @NonNull LoopState loopState) {
        double error = target - motionState.getVelocity();
        return updateWithError(error, motionState, loopState);
    }

    @Override
    public double updateWithError(double error, MotionState motionState, LoopState loopState) {
        if (loopState.getDt() == 0) {
            throw new ArithmeticException("LoopState dt cannot be zero for PIDFFVelocityController");
        }
        integral += error * loopState.getDt();
        double derivative = (error - lastError) / loopState.getDt();
        double pid = coef.getKp() * error
                + coef.getKi() * integral
                + coef.getKd() * derivative;
        double ff =
                (coef.getKs() * Math.signum(error)
                        + coef.getKv() * motionState.getVelocity()
                        + coef.getKa() * motionState.getAcceleration()) / loopState.getBatteryVoltageFactor();
        double output = pid + ff;
        lastError = error;
        return output;
    }


}
