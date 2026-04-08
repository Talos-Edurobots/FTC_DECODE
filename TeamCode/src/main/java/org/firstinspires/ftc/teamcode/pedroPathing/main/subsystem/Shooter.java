package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private boolean isRunning = true;
    public static double alpha = .2;
    public static double dropThreshold = 100;
    public static double debounceTime = .5;
    public double filteredVelocity = 0;
    private double lastFilteredVel;
    private final ElapsedTime loopTimer = new ElapsedTime();

    public boolean isImpactDetected() {
        return impactDetected;
    }

    private boolean impactDetected = false;
    ElapsedTime impactTimer = new ElapsedTime();
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
        loopTimer.reset();
    }

    public void update() {
        dt = loopTimer.seconds();
        loopTimer.reset();
        MotorConfig.setDt(dt);
        MotorConfig.setBatteryVoltage(getBatteryVoltage());
        motor.setVelocityTicksPerSecond(targetVelocity);
        calculateFilteredVelocity();
        setVelocity();
    }
    public double getImpactTime() {
        return impactTimer.seconds();
    }
    public void calculateFilteredVelocity() {
        filteredVelocity = alpha * motor.getVelocity() + (1 - alpha) * filteredVelocity;
        if (dt <= 0) {
            lastFilteredVel = filteredVelocity;
            return;
        }

        double delta = (filteredVelocity - lastFilteredVel) / dt;
        // Detect shot
        if (!impactDetected && -delta > dropThreshold) {
            impactDetected = true;
            impactTimer.reset();
        }

        // Debounce reset
        if (impactDetected && impactTimer.seconds() > debounceTime) {
            impactDetected = false;
        }
        lastFilteredVel = filteredVelocity;
    }
    public void run(boolean runMotor) {
        this.isRunning = runMotor;
    }
    public boolean getRun(){
        return isRunning;
    }
    public void changeState(){
        this.isRunning ^= true;
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
    public void setVelocity() {
        if (isRunning) motor.updateVelocityPIDF();
        else floatShooter();
    }
    public void setPower(double power) {
        motor.setPower(power);
    }
    public void floatShooter() {
        motor.setPower(0);
    }
    public MotorConfig getInstance() {
        return motor;
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getCurrent() {
        return motor.getCurrent();
    }

    private double getBatteryVoltage() {
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwmap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                minVoltage = Math.min(minVoltage, voltage);
            }
        }
        return minVoltage < Double.POSITIVE_INFINITY ? minVoltage : 12.0;
    }

}
