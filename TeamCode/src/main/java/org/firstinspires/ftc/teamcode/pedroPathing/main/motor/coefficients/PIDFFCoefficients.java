package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients;

public class PIDFFCoefficients {
    private double kp, ki, kd, ks, kv, ka;

    public double getKp() {
        return kp;
    }

    public double getKa() {
        return ka;
    }

    public double getKv() {
        return kv;
    }

    public double getKs() {
        return ks;
    }

    public double getKd() {
        return kd;
    }

    public double getKi() {
        return ki;
    }

    PIDFFCoefficients(double kp, double ki, double kd, double ks, double kv, double ka) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
    }
}
