package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients;

public class PIDFFCoefficients {
    private double kp, ki, kd, ks, kv, ka;

    public double kp() {
        return kp;
    }

    public double ka() {
        return ka;
    }

    public double kv() {
        return kv;
    }

    public double ks() {
        return ks;
    }

    public double kd() {
        return kd;
    }

    public double ki() {
        return ki;
    }

    public PIDFFCoefficients(double kp, double ki, double kd, double ks, double kv, double ka) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
    }
    public PIDFFCoefficients(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, 0, 0);
    }
}
