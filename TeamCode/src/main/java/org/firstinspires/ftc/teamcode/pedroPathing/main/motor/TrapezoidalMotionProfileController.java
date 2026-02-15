package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.MotionProfilingCoefficients;

class TrapezoidalMotionProfileController implements MotorController{
    MotionProfilingCoefficients coef;
    PIDFFPositionController pidFFController;
    private double xRef = 0.0, vRef = 0.0, aRef = 0.0;
    public TrapezoidalMotionProfileController(MotionProfilingCoefficients coefficients) {;
        this.coef = coefficients;
        pidFFController = new PIDFFPositionController(coef.getPidCoef());
    }

    @Override
    public double update(double target, @NonNull MotionState motionState, LoopState loopState) {
        double position = motionState.position;
        double velocity = motionState.velocity;

        double remaining = target - xRef;

        double stoppingDistance =
                (vRef * vRef) / (2.0 * coef.getMaxAcceleration());

        if (Math.abs(remaining) <= stoppingDistance) {
            aRef = -Math.signum(velocity) * coef.getMaxAcceleration();
        } else {
            aRef = Math.signum(remaining) *coef.getMaxAcceleration();
        }

        vRef += aRef * loopState.dt;
        vRef = Range.clip(vRef, -coef.getMaxVelocity(), coef.getMaxVelocity());

        xRef += vRef * loopState.dt;

        if (Math.signum(target - xRef)
                != Math.signum(remaining)) {
            xRef = target;
            vRef = 0.0;
            aRef = 0.0;
        }


        double pidVolts = pidFFController.update(xRef, motionState, loopState);

        double ffVolts =
                coef.getPidCoef().getKs() * Math.signum(vRef) +
                        coef.getPidCoef().getKv() * vRef +
                        coef.getPidCoef().getKa() * aRef;

        double power =
                (pidVolts + ffVolts) / loopState.batteryVoltage;
        return power;
    }
}
