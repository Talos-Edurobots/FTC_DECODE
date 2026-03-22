package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.LoopState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.MotionProfilingCoefficients;

public class TrapezoidalMotionProfileController implements MotorController{
    MotionProfilingCoefficients coef;
    PIDFFPositionController pidFFController;
    private double xRef = 0.0, vRef = 0.0, aRef = 0.0;
    public TrapezoidalMotionProfileController(MotionProfilingCoefficients coefficients) {;
        this.coef = coefficients;
        pidFFController = new PIDFFPositionController(coef.getPidCoef());
    }
    public void init(MotionState currentState) {
        xRef = currentState.getPosition();
        vRef = 0.0;
        aRef = 0.0;
        pidFFController.reset();
    }

    @Override
    public double updateWithError(double error, MotionState motionState, LoopState loopState) {
        return 0;
    }

    @Override
    public double update(double target, @NonNull MotionState motionState, LoopState loopState) {
        double position = motionState.getPosition();
        double velocity = motionState.getVelocity();

        double remaining = target - xRef;

        double stoppingDistance =
                (vRef * vRef) / (2.0 * coef.getMaxAcceleration());

        if (Math.abs(remaining) <= stoppingDistance) {
            aRef = -Math.signum(velocity) * coef.getMaxAcceleration();
        } else {
            aRef = Math.signum(remaining) *coef.getMaxAcceleration();
        }

        vRef += aRef * loopState.getDt();
        vRef = Range.clip(vRef, -coef.getMaxVelocity(), coef.getMaxVelocity());

        xRef += vRef * loopState.getDt();

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
                (pidVolts + ffVolts) / loopState.getBatteryVoltageFactor();
        return power;
    }
}
