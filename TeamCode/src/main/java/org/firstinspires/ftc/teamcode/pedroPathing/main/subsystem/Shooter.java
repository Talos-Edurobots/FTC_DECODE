package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.LoopState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.facade.OpenLoopMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.facade.VelocityControlledMotor;

@Configurable
public class Shooter {
    private final HardwareMap hwmap;
    private VelocityControlledMotor shooterMotor;
    private OpenLoopMotor shooterFollowerMotor;
    private Servo hoodServo;
    private double dt = 0;
    private boolean isRunning = true;
    public static double alpha = .2;
    public static double dropThreshold = 100;
    public static double debounceTime = .5;
    public double filteredVelocity = 0;
    private double lastFilteredVel;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final LoopState loopState = new LoopState();

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
    public void init() {
        shooterMotor = VelocityControlledMotor.fromLegacyTickCoefficients(
                RobotConstants.SHOOTER_MOTOR_NAME,
                RobotConstants.SHOOTER_MOTOR_TYPE,
                RobotConstants.SHOOTER_MOTOR_DIRECTION,
                RobotConstants.SHOOTER_ZERO_POWER_BEHAVIOR,
                RobotConstants.SHOOTER_VELOCITY_PIDF,
                RobotConstants.SHOOTER_LIMITS
        );
        shooterMotor.init(hwmap);
        shooterFollowerMotor = new OpenLoopMotor(
                RobotConstants.SHOOTER_FOLLOWER_MOTOR_NAME,
                RobotConstants.SHOOTER_FOLLOWER_DIRECTION,
                RobotConstants.SHOOTER_FOLLOWER_ZERO_POWER_BEHAVIOR,
                RobotConstants.SHOOTER_LIMITS
        );
        shooterFollowerMotor.init(hwmap);

        hoodServo = hwmap.get(Servo.class, RobotConstants.LEFT_SERVO_NAME);
        hoodServo.setPosition(.5);
        hoodServo.setDirection(Servo.Direction.FORWARD);
        loopTimer.reset();
    }

    public void update() {
        dt = loopTimer.seconds();
        loopTimer.reset();
        loopState.set(dt, 1.0 / getBatteryVoltage());
        shooterMotor.setTargetVelocityTicksPerSecond(targetVelocity);
        calculateFilteredVelocity();
        setVelocity();
    }
    public double getImpactTime() {
        return impactTimer.seconds();
    }
    public void calculateFilteredVelocity() {
        filteredVelocity = alpha * shooterMotor.getMeasuredVelocityTicksPerSecond() + (1 - alpha) * filteredVelocity;
        if (dt <= 0) {
            lastFilteredVel = filteredVelocity;
            return;
        }

        double delta = (filteredVelocity - lastFilteredVel) / dt;
        if (!impactDetected && -delta > dropThreshold) {
            impactDetected = true;
            impactTimer.reset();
        }

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
        return Math.abs(targetVelocity - shooterMotor.getMeasuredVelocityTicksPerSecond()) > 70;
    }
    public void setVelocity() {
        if (isRunning) {
            shooterMotor.update(loopState);
            shooterFollowerMotor.setPower(shooterMotor.getPower());
        }
        else floatShooter();
    }
    public void setPower(double power) {
        shooterMotor.setPower(power);
        shooterFollowerMotor.setPower(power);
    }
    public void floatShooter() {
        setPower(0);
    }

    public double getVelocity() {
        return shooterMotor.getMeasuredVelocityTicksPerSecond();
    }

    public double getPower() {
        return shooterMotor.getPower();
    }

    public double getCurrent1() {
        return shooterMotor.getCurrentAmps();
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
