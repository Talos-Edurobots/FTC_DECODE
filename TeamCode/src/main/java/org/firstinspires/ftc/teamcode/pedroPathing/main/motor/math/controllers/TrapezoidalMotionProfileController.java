package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.MotionProfilingCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.AngularVelocity;

public class TrapezoidalMotionProfileController implements MotorController{
    private final MotionProfilingCoefficients coef;
    private final PIDFFPositionController pidFFController;
    private Angle xRef = Angle.fromRadians(0);
    private AngularVelocity vRef = AngularVelocity.fromRadPerSec(0);
    private double aRef = 0.0; // rad/s^2

    public TrapezoidalMotionProfileController(MotionProfilingCoefficients coefficients) {
        this.coef = coefficients;
        pidFFController = new PIDFFPositionController(coef.getPidCoef());
    }

    @Override
    public double update(MotionState target, @NonNull MotionState motionState, double dt) {
        return update(target, motionState, dt, 1.0);
    }

    public double update(MotionState target,
                         @NonNull MotionState motionState,
                         double dt,
                         double feedforwardScale) {
        if (dt == 0.0) {
            throw new ArithmeticException("dt cannot be zero");
        }

        double position = motionState.getPosition().toRadians();
        double velocity = motionState.getVelocity().toRadPerSec();

        double targetRad = target.getPosition().toRadians();
        double xRefRad = xRef.toRadians();
        double vRefRad = vRef.toRadPerSec();

        double remaining = targetRad - xRefRad;

        double stoppingDistance =
                (vRefRad * vRefRad) / (2.0 * coef.getMaxAcceleration());

        if (Math.signum(vRefRad) == Math.signum(remaining) &&
                Math.abs(remaining) <= stoppingDistance) {
            aRef = -Math.signum(velocity) * coef.getMaxAcceleration();
        } else {
            aRef = Math.signum(remaining) *coef.getMaxAcceleration();
        }
        vRefRad += aRef * dt;
        vRefRad = Range.clip(vRefRad,
                -coef.getMaxVelocity(),
                coef.getMaxVelocity());

        xRefRad += vRefRad * dt;

        if (Math.signum(targetRad - xRefRad) != Math.signum(remaining)) {
            xRefRad = targetRad;
            vRefRad = 0.0;
            aRef = 0.0;
        }

        xRef = Angle.fromRadians(xRefRad);
        vRef = AngularVelocity.fromRadPerSec(vRefRad);
        MotionState refState = new MotionState(xRef, vRef, aRef);
        double staticFeedforwardSign = Math.signum(vRefRad);
        if (staticFeedforwardSign == 0.0) {
            staticFeedforwardSign = Math.signum(targetRad - position);
        }
        return pidFFController.update(
                refState,
                motionState,
                dt,
                feedforwardScale,
                staticFeedforwardSign
        );
    }

    public MotionState getReferenceState() {
        return new MotionState(xRef, vRef, aRef);
    }

    public void reset(MotionState measuredState) {
        xRef = measuredState.getPosition();
        vRef = measuredState.getVelocity();
        aRef = measuredState.getAcceleration();
        pidFFController.reset();
    }

    @Override
    public void reset() {
        reset(new MotionState(
                Angle.fromRadians(0.0),
                AngularVelocity.fromRadPerSec(0.0),
                0.0
        ));
    }
}
