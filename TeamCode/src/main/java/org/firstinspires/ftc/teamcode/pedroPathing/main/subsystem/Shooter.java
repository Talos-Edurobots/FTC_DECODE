package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.main.RobotConstants;

public class Shooter {
    private HardwareMap hwmap;
    private DcMotorEx motor;
    private Servo hoodServo;
    private double integralSum = 0;
    private double lastError = 0;
    private double dt = 0;
    private boolean runMotor = true;
    private double targetVelocity = 0;
    public Shooter(HardwareMap hwmap) {
        this.hwmap = hwmap;
    }
    public void init(){
        motor = hwmap.get(DcMotorEx.class, RobotConstants.SHOOTER_NAME);
        motor.setDirection(RobotConstants.SHOOTER_DIRECTION);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hoodServo = hwmap.get(Servo.class, RobotConstants.LEFT_SERVO_NAME);
        hoodServo.setPosition(0);
        hoodServo.setDirection(Servo.Direction.FORWARD);
    }
    public void update(double targetVelocity, double dt) {
        this.targetVelocity = targetVelocity;
        this.dt = dt;
        if (runMotor) {
            setVelocity();
        } else {
            floatShooter();
        }
    }
    public void run(boolean runMotor) {
        this.runMotor = runMotor;
    }
    public boolean getRun(){
        return runMotor;
    }
    public void changeRun(){
        this.runMotor ^= true;
    }
    public void setHoodAngle(double pwm) {
        hoodServo.setPosition(pwm);
    }
    public double getHoodAngle() {
        return hoodServo.getPosition();
    }
    public boolean isBusy () {
        return Math.abs(targetVelocity - motor.getVelocity()) < 70;
    }
    private void setVelocity() {
        // Set velocity via triggers
        double currentVelocity = motor.getVelocity();
        double error = targetVelocity - currentVelocity;
        double derivative = (error - lastError) / dt;
        integralSum += error * dt;

        if (runMotor) {
            motor.setPower(
                RobotConstants.SHOOTER_K_P * error +
                RobotConstants.SHOOTER_K_I * integralSum +
                RobotConstants.SHOOTER_K_D * derivative +
                RobotConstants.SHOOTER_K_S * Math.signum(targetVelocity) +
                RobotConstants.SHOOTER_K_V * targetVelocity
            );
        }
        else {
            motor.setPower(0);
        }
    }
    private void floatShooter() {
        motor.setPower(0);
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }
}
