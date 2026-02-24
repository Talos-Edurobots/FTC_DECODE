package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;

@Configurable
public class Shooter {
    private HardwareMap hwmap;
    private MotorConfig motor;
    private Servo hoodServo;
    private double integralSum = 0;
    private double lastError = 0;
    private double dt = 0;
    private boolean runMotor = true;
    public static double alpha = .2;
    public double filteredVelocity = 0;
    public double getTargetVelocity() {
        return targetVelocity;
    }

    public static void setTargetVelocity(double targetVelocity) {
        Shooter.targetVelocity = targetVelocity;
    }

    public static double targetVelocity = 2400;
    public Shooter(HardwareMap hwmap) {
        this.hwmap = hwmap;
    }
    public void init(){
        motor = RobotConstants.SHOOTER_CONFIG;
        motor.init(hwmap);

        hoodServo = hwmap.get(Servo.class, RobotConstants.LEFT_SERVO_NAME);
        hoodServo.setPosition(.5);
        hoodServo.setDirection(Servo.Direction.FORWARD);
    }

    public void update(double dt) {
        this.dt = dt;
        calculateFilteredVelocity();
        if (runMotor) {
            setVelocity();
        } else {
            floatShooter();
        }
    }
    void calculateFilteredVelocity() {
        filteredVelocity = alpha * motor.getVelocity() + (1 - alpha) * filteredVelocity;
    }
    public void run(boolean runMotor) {
        this.runMotor = runMotor;
    }
    public boolean getRun(){
        return runMotor;
    }
    public void changeState(){
        this.runMotor ^= true;
    }
    public void setHoodAngle(double pwm) {
        hoodServo.setPosition(Range.clip(pwm, 0, .5));
    }
    public double getHoodAngle() {
        return hoodServo.getPosition();
    }
    public boolean isBusy () {
        return Math.abs(targetVelocity - motor.getVelocity()) > 70;
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
        return motor.getCurrent();
    }

}
