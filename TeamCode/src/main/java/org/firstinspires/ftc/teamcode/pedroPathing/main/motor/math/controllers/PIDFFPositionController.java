package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;

public class PIDFFPositionController implements MotorController{

    private double integral = 0.0;

    private PIDFFCoefficients coef;

    private boolean antiWindup;
    private double integralLimit;

    public PIDFFPositionController(PIDFFCoefficients coef) {
        this(coef, false, Double.POSITIVE_INFINITY);
    }

    public PIDFFPositionController(PIDFFCoefficients coef,
                                   boolean antiWindup,
                                   double integralLimit) {
        this.coef = coef;
        this.antiWindup = antiWindup;
        this.integralLimit = integralLimit;
    }

    @Override
    public double update(MotionState ref,
                         MotionState current,
                         double dt) {
        double positionError = ref.getPosition().toRadians()
                - current.getPosition().toRadians();
        return update(ref, current, dt, 1.0, Math.signum(positionError));
    }

    public double update(MotionState ref,
                         MotionState current,
                         double dt,
                         double feedforwardScale,
                         double staticFeedforwardSign) {
        if (dt == 0) {
            throw new ArithmeticException("dt cannot be zero");
        }

        // --- Extract physical values ---
        double xRef = ref.getPosition().toRadians();
        double vRef = ref.getVelocity().toRadPerSec();
        double aRef = ref.getAcceleration();

        double x = current.getPosition().toRadians();
        double v = current.getVelocity().toRadPerSec();

        // --- Errors ---
        double error = xRef - x;
        double velError = vRef - v;

        // --- Integral ---
        integral += error * dt;

        if (antiWindup) {
            integral = Math.max(-integralLimit,
                    Math.min(integralLimit, integral));
        }

        // --- PID ---
        double pid =
                coef.kp() * error +
                        coef.ki() * integral +
                        coef.kd() * velError;

        // --- Feedforward ---
        double ff =
                coef.ks() * staticFeedforwardSign +
                        coef.kv() * vRef +
                        coef.ka() * aRef;

        return pid + ff * feedforwardScale;
    }

    public void reset() {
        integral = 0.0;
    }
}
