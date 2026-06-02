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
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.ShooterTelemetrySnapshot;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryCollector;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryCostClass;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryMode;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryProvider;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.ThrottledValue;

@Configurable
public class Shooter implements TelemetryProvider {
    private static final double VOLTAGE_SAMPLE_INTERVAL_SECONDS = 0.1;
    private static final double SERVO_WRITE_EPSILON = 1e-4;

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
    private final ThrottledValue<Double> currentSampler = new ThrottledValue<>(0.1);
    private final ThrottledValue<Double> followerCurrentSampler = new ThrottledValue<>(0.1);
    private final ElapsedTime batteryVoltageTimer = new ElapsedTime();
    private double cachedBatteryVoltage = 12.0;
    private double lastHoodPosition = Double.NaN;

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
                RobotConstants.SHOOTER_LIMITS,
                RobotConstants.SHOOTER_POWER_WRITE_EPSILON
        );
        shooterMotor.init(hwmap);
        shooterFollowerMotor = new OpenLoopMotor(
                RobotConstants.SHOOTER_FOLLOWER_MOTOR_NAME,
                RobotConstants.SHOOTER_FOLLOWER_DIRECTION,
                RobotConstants.SHOOTER_FOLLOWER_ZERO_POWER_BEHAVIOR,
                RobotConstants.SHOOTER_LIMITS,
                RobotConstants.SHOOTER_POWER_WRITE_EPSILON
        );
        shooterFollowerMotor.init(hwmap);

        hoodServo = hwmap.get(Servo.class, RobotConstants.LEFT_SERVO_NAME);
        applyHoodPosition(.5);
        hoodServo.setDirection(Servo.Direction.FORWARD);
        cachedBatteryVoltage = readBatteryVoltage();
        batteryVoltageTimer.reset();
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
    public void updateOpenLoopFeedforward() {
        dt = loopTimer.seconds();
        loopTimer.reset();
        calculateFilteredVelocity();
        isRunning = true;
        setPower(calculateOpenLoopFeedforwardPower(targetVelocity));
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
        applyHoodPosition(Range.clip(pwm, 0, .5));
    }
    public double getHoodAngle() {
        return Double.isNaN(lastHoodPosition) ? hoodServo.getPosition() : lastHoodPosition;
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

    private double calculateOpenLoopFeedforwardPower(double targetVelocityTicksPerSecond) {
        return (RobotConstants.SHOOTER_VELOCITY_PIDF.ks() * Math.signum(targetVelocityTicksPerSecond)
                + RobotConstants.SHOOTER_VELOCITY_PIDF.kv() * targetVelocityTicksPerSecond)
                / getBatteryVoltage();
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

    public double getCurrent2() {
        return shooterFollowerMotor.getCurrentAmps();
    }

    public ShooterTelemetrySnapshot getTelemetrySnapshot(TelemetryMode mode, double nowSeconds) {
        Double currentAmps = mode.includes(TelemetryMode.DEBUG)
                ? currentSampler.get(nowSeconds, shooterMotor::getCurrentAmps)
                : null;
        Double followerCurrentAmps = mode.includes(TelemetryMode.DEBUG)
                ? followerCurrentSampler.get(nowSeconds, shooterFollowerMotor::getCurrentAmps)
                : null;
        return new ShooterTelemetrySnapshot(
                targetVelocity,
                shooterMotor.getMeasuredVelocityTicksPerSecond(),
                filteredVelocity,
                shooterMotor.getPower(),
                getHoodAngle(),
                isRunning,
                isBusy(),
                impactDetected,
                shooterMotor.getHardware().isOverCurrent(),
                currentAmps,
                followerCurrentAmps,
                getBatteryVoltage()
        );
    }

    @Override
    public void collectTelemetry(TelemetryCollector collector, TelemetryMode mode) {
        ShooterTelemetrySnapshot snapshot = getTelemetrySnapshot(mode, collector.getNowSeconds());

        collector.add("shooter", "running", snapshot.running, TelemetryMode.COMPETITION,
                TelemetryCostClass.CHEAP);
        collector.add("shooter", "busy", snapshot.busy, TelemetryMode.COMPETITION,
                TelemetryCostClass.CHEAP);
        collector.add("shooter", "impact_detected", snapshot.impactDetected,
                TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("shooter", "over_current", snapshot.overCurrent,
                TelemetryMode.COMPETITION, TelemetryCostClass.BULK_CACHED);
        collector.add("shooter", "target_tps", snapshot.targetVelocityTicksPerSecond,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("shooter", "measured_tps", snapshot.measuredVelocityTicksPerSecond,
                TelemetryMode.DEBUG, TelemetryCostClass.BULK_CACHED);
        collector.add("shooter", "filtered_tps", snapshot.filteredVelocityTicksPerSecond,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("shooter", "power", snapshot.appliedPower,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("shooter", "hood", snapshot.hoodAngle,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("shooter", "current_amps", snapshot.currentAmps,
                TelemetryMode.DEBUG, TelemetryCostClass.NON_BULK);
        collector.add("shooter", "follower_current_amps", snapshot.followerCurrentAmps,
                TelemetryMode.DEBUG, TelemetryCostClass.NON_BULK);
        collector.add("shooter", "battery_voltage", snapshot.batteryVoltage,
                TelemetryMode.DEBUG, TelemetryCostClass.NON_BULK);
    }

    public double getBatteryVoltage() {
        if (batteryVoltageTimer.seconds() < VOLTAGE_SAMPLE_INTERVAL_SECONDS) {
            return cachedBatteryVoltage;
        }
        cachedBatteryVoltage = readBatteryVoltage();
        batteryVoltageTimer.reset();
        return cachedBatteryVoltage;
    }

    private void applyHoodPosition(double position) {
        if (Double.isNaN(lastHoodPosition)
                || Math.abs(position - lastHoodPosition) >= SERVO_WRITE_EPSILON) {
            hoodServo.setPosition(position);
            lastHoodPosition = position;
        }
    }

    private double readBatteryVoltage() {
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
