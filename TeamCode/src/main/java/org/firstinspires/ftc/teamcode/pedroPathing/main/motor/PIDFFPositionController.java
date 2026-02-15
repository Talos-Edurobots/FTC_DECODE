package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;

class PIDFFPositionController implements MotorController {
    double lastError, integral;

    PIDFFCoefficients coef;
    boolean antiWindup = false;
    double integralAntiWindupLimit = Double.POSITIVE_INFINITY;

    public PIDFFPositionController(PIDFFCoefficients coefficients) {
        this.coef = coefficients;
    }
    public PIDFFPositionController(PIDFFCoefficients coefficients, boolean antiWindup, double integralAntiWindupLimit) {
        this.coef = coefficients;
        this.antiWindup = antiWindup;
        this.integralAntiWindupLimit = integralAntiWindupLimit;
    }

    @Override
    public double update(double target, @NonNull MotionState motionState, LoopState loopState) {
        double error = target - motionState.position;
        return  updateWithError(error, motionState, loopState);
    }
    public double updateWithError(double error, @NonNull MotionState motionState, LoopState loopState) {
        double pid = coef.getKp() * error
                + coef.getKi() * integral
                + coef.getKd() * (-motionState.velocity);

        double ff = coef.getKs() * Math.signum(-error);
        double output = pid + ff;
        if (!antiWindup || Math.abs(output) < integralAntiWindupLimit) {
            integral += error * loopState.dt;
        }
        lastError = error;
        return output;
    }

    public void reset() {
        lastError = 0.0;
        integral = 0.0;
    }
    public void updateCoefficients(PIDFFCoefficients coefficients) {
        this.coef = coefficients;
    }

    public PIDFFCoefficients getCoefficients() {
        return coef;
    }
}
