package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;

class PIDFFPositionController {
    double lastError, integral;

    PIDFFCoefficients coef;

    public PIDFFPositionController(PIDFFCoefficients coefficients) {
        this.coef = coefficients;
    }


    public double update(double target, MotionState motionState, LoopState loopState) {
        double error = target - motionState.position;
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

    public void updateCoefficients(PIDFFCoefficients coefficients) {
        this.coef = coefficients;
    }

    public PIDFFCoefficients getCoefficients() {
        return coef;
    }
}
