package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;

class PIDFFVelocityController {
    double lastError, integral;
    PIDFFCoefficients coef;

    public PIDFFVelocityController(PIDFFCoefficients coefficients) {
        this.coef = coefficients;
    }


    public double update(double target, MotionState motionState, LoopState loopState) {
        double error = target - motionState.velocity;
        double derivative = (error - lastError) / loopState.dt;
        integral += error * loopState.dt;
        double pid = coef.getKp() * error
                + coef.getKi() * integral
                + coef.getKd() * derivative;
        double ff =
                (coef.getKs() * Math.signum(target)
                + coef.getKv() * motionState.velocity
                + coef.getKa() * motionState.acceleration) / loopState.batteryVoltage;
        double output = pid + ff;
        lastError = error;
        return output;
    }
}
