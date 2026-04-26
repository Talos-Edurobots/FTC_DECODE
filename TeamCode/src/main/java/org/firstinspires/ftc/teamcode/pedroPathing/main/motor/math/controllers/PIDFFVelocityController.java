package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.controllers;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotionState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.PIDFFCoefficients;

public class PIDFFVelocityController implements MotorController{

    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean hasLastError = false;

    private PIDFFCoefficients coef;

    private boolean antiWindup;
    private double integralLimit;

    public PIDFFVelocityController(PIDFFCoefficients coef) {
        this(coef, false, Double.POSITIVE_INFINITY);
    }

    public PIDFFVelocityController(PIDFFCoefficients coef,
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
        return update(ref, current, dt, 1.0);
    }

    public double update(MotionState ref,
                         MotionState current,
                         double dt,
                         double feedforwardScale) {
        if (dt == 0) {
            throw new ArithmeticException("dt cannot be zero");
        }

        // --- Extract values ---
        double vRef = ref.getVelocity().toRadPerSec();
        double aRef = ref.getAcceleration();

        double v = current.getVelocity().toRadPerSec();

        // --- Error ---
        double error = vRef - v;

        // --- Integral ---
        integral += error * dt;

        if (antiWindup) {
            integral = Math.max(-integralLimit,
                    Math.min(integralLimit, integral));
        }

        double derivative = 0.0;
        if (hasLastError) {
            derivative = (error - lastError) / dt;
        }
        lastError = error;
        hasLastError = true;

        // --- PID (velocity loop) ---
        double pid =
                coef.kp() * error +
                        coef.ki() * integral +
                        coef.kd() * derivative;

        // --- Feedforward ---
        double ff =
                coef.ks() * Math.signum(vRef) +
                        coef.kv() * vRef +
                        coef.ka() * aRef;

        return pid + ff * feedforwardScale;
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        hasLastError = false;
    }
}
