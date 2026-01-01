package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.main.config.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.config.RobotConstants;

@Configurable
public class Shooter {
    private HardwareMap hwmap;
    private MotorConfig motor;
    private LED greenLED;
    private LED redLED;
    private Servo hoodServo;
    private double integralSum = 0;
    private double lastError = 0;
    private double dt = 0;
    private boolean runMotor = true;

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public static double targetVelocity = 2400;
    public Shooter(HardwareMap hwmap) {
        this.hwmap = hwmap;
    }
    public void init(){
        motor = RobotConstants.SHOOTER_CONFIG;
        motor.init(hwmap);

        hoodServo = hwmap.get(Servo.class, RobotConstants.LEFT_SERVO_NAME);
        hoodServo.setPosition(0);
        hoodServo.setDirection(Servo.Direction.FORWARD);

        greenLED = hwmap.get(LED.class, RobotConstants.SHOOTER_LED_GREEN);
        redLED = hwmap.get(LED.class, RobotConstants.SHOOTER_LED_RED);
    }

    public void update(double dt) {
        this.dt = dt;
        if (runMotor) {
            setVelocity();
            if (isBusy()) {
                setLEDsNotReady();
            } else {
                setLEDsReady();
            }
        } else {
            floatShooter();
            setLEDsOff();
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
        return motor.getCurrent();
    }
    public void setLEDsOff() {
        greenLED.enable(false);
        redLED.enable(false);
    }
    public void setLEDsReady() {
        greenLED.enable(true);
        redLED.enable(false);
    }
    public void setLEDsNotReady() {
        greenLED.enable(false);
        redLED.enable(true);
    }
}
