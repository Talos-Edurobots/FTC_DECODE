package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.LoopState;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MetaMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorCoefficientScaler;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.coefficients.MotionProfilingCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.facade.ProfiledPositionMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units.EncoderConverter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryCollector;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryCostClass;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryMode;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryPublishPolicy;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryProvider;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.ThrottledValue;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TurretTelemetrySnapshot;

@Configurable
public class Turret implements TelemetryProvider {
    private static final double MAX_REASONABLE_LOOP_DT_SECONDS = 0.25;
    private static final PositionAimLut RED_POSITION_AIM_LUT = new PositionAimLut(
            PositionAimLut.sample(72.0, 72.0, 144.0, 144.0),
            PositionAimLut.sample(36.5, 131.5, 144.0, 133.9),
            PositionAimLut.sample(96.5, 9.6, 133.9, 144.0),
            PositionAimLut.sample(57.6, 20.2, 139.2, 144.0),
            PositionAimLut.sample(73.4, 9.1, 137.8, 144.0),
            PositionAimLut.sample(50.4, 108.0, 144.0, 135.8),
            PositionAimLut.sample(85.4, 97.9, 139.2, 144.0),
            PositionAimLut.sample(104.26,131.33,144.0,135.07)
    );
    private static final PositionAimLut BLUE_POSITION_AIM_LUT = new PositionAimLut(
            // Add calibrated blue-alliance samples here: sample(robotX, robotY, aimX, aimY)
    );

    private enum ControlMode {
        PROFILED,
        MANUAL_PID
    }

    static double maxPower = RobotConstants.TURRET_LIMITS.getMaxPower();
    static double kp = RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getPidCoef().kp();

    public static void setMaxPower(double maxPower) {
        Turret.maxPower = maxPower;
    }

    public static void setKp(double kp) {
        Turret.kp = kp;
    }

    public static void setKi(double ki) {
        Turret.ki = ki;
    }

    public static void setKd(double kd) {
        Turret.kd = kd;
    }

    public static void setKs(double ks) {
        Turret.ks = ks;
    }

    public static void setKv(double kv) {
        Turret.kv = kv;
    }

    public static void setKa(double ka) {
        Turret.ka = ka;
    }

    public static void setMaxVel(double maxVel) {
        Turret.maxVel = maxVel;
    }

    public static void setMaxAcc(double maxAcc) {
        Turret.maxAcc = maxAcc;
    }

    public static void setMaxDec(double maxDec) {
        Turret.maxDec = maxDec;
    }

    static double ki = RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getPidCoef().ki();
    static double kd = RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getPidCoef().kd();
    static double ks = RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getPidCoef().ks();
    static double kv = RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getPidCoef().kv();
    static double ka = RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getPidCoef().ka();
    static double maxVel = RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getMaxVelocity();
    static double maxAcc = RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getMaxAcceleration();
    static double maxDec = RobotConstants.TURRET_CONFIGURABLE_PROFILE_DEFAULTS.getMaxDeceleration();
    public static double targetToleranceDegrees = 1.0;

    public static double getMaxPower() {
        return maxPower;
    }

    public static double getKp() {
        return kp;
    }

    public static double getKi() {
        return ki;
    }

    public static double getKd() {
        return kd;
    }

    public static double getKs() {
        return ks;
    }

    public static double getKv() {
        return kv;
    }

    public static double getKa() {
        return ka;
    }

    public static double getMaxVel() {
        return maxVel;
    }

    public static double getMaxAcc() {
        return maxAcc;
    }

    public static double getMaxDec() {
        return maxDec;
    }

    static double manualMaxPower = .2, ramp = 1;
    public static double movingShotLeadFactor = 0.01;
    public static boolean positionAimLutEnabled = true;
    public static int positionAimLutNeighborCount = 3;
    private final HardwareMap hwmap;
    private final MetaMotor turretHardware = new MetaMotor();
    private final EncoderConverter encoderConverter =
            new EncoderConverter(RobotConstants.TURRET_MOTOR_TYPE);
    private final MotionProfilingCoefficients turretProfileCoefficients =
            MotorCoefficientScaler.fromLegacyTickSpace(
                    RobotConstants.TURRET_PROFILE_COEFFICIENTS,
                    encoderConverter
            );
    private final ProfiledPositionMotor turret = new ProfiledPositionMotor(
            turretHardware,
            RobotConstants.TURRET_MOTOR_TYPE,
            turretProfileCoefficients,
            RobotConstants.TURRET_LIMITS.getMaxPower()
    );
    static final Pose RED_GOAL_POSE = new Pose(144, 137);
    final Pose BLUE_GOAL_POSE = new Pose(0, 140);
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final LoopState loopState = new LoopState();
    private final ThrottledValue<Double> currentSampler = new ThrottledValue<>(0.1);

    private ControlMode controlMode = ControlMode.PROFILED;
    private double angleToGoal;
    private double targetMechanismAngleRadians = 0.0;
    private double manualExtraPower = 0.0;
    private double manualIntegral = 0.0;
    private double lastManualError = 0.0;
    private Double lastAimPointX = null;
    private Double lastAimPointY = null;
    private boolean lastAimPointWasVirtual = false;

    public Turret(HardwareMap hwmap) {
        this.hwmap = hwmap;
        turretHardware.hwName(RobotConstants.TURRET_MOTOR_NAME);
        turretHardware.direction(RobotConstants.TURRET_MOTOR_DIRECTION);
        turretHardware.zeroPowerBehavior(RobotConstants.TURRET_ZERO_POWER_BEHAVIOR);
        turretHardware.maxPower(RobotConstants.TURRET_LIMITS.getMaxPower());
        turretHardware.currentAlert(RobotConstants.TURRET_LIMITS.getCurrentAlertAmps());
        turretHardware.powerWriteEpsilon(RobotConstants.TURRET_POWER_WRITE_EPSILON);
    }

    public void faceForward() {
        setAngleRadians(0);
    }

    public void setAngleRadians(double angleRadians) {
        targetMechanismAngleRadians = Range.clip(
                angleRadians,
                RobotConstants.TURRET_MIN_ANGLE_RADIANS,
                RobotConstants.TURRET_MAX_ANGLE_RADIANS
        );
        turret.setTargetAngle(Angle.fromRadians(toRawMotorRadians(targetMechanismAngleRadians)));
        if (controlMode != ControlMode.PROFILED) {
            turret.resetController();
            controlMode = ControlMode.PROFILED;
        }
    }

    public double getAngleToGoal(){
        return angleToGoal;
    }

    public double getMeasuredAngleRadians() {
        return rawMotorRadiansToMechanismRadians(turret.getMeasuredAngle().toRadians());
    }

    public void lookToGoal(Pose pose, boolean isRed) {
        if (pose == null) {
            return;
        }
        Pose aimTarget = getAimTargetPose(pose, isRed);
        angleToGoal = computeTurretAngleRadians(pose, aimTarget);
        setAngleRadians(angleToGoal);
    }

    public void lookToGoalWhileMoving(Pose pose, Vector velocity, boolean isRed) {
        lookToGoalWhileMoving(pose, velocity, movingShotLeadFactor, isRed);
    }

    public void lookToGoalWhileMoving(Pose pose, Vector velocity, double leadFactor, boolean isRed) {
        if (pose == null || velocity == null) return;

        Pose target = isRed ? RED_GOAL_POSE : BLUE_GOAL_POSE;
        double distance = target.distanceFrom(pose);
        Pose compensatedPose = new Pose(
                pose.getX() + leadFactor * velocity.getXComponent() * distance,
                pose.getY() + leadFactor * velocity.getYComponent() * distance,
                pose.getHeading()
        );

        lookToGoal(compensatedPose, isRed);
    }
    public void init() {
        applyProfileConfigurables();
        turret.init(hwmap);
        turret.setAngleLimits(
                Angle.fromRadians(toRawMotorRadians(RobotConstants.TURRET_MIN_ANGLE_RADIANS)),
                Angle.fromRadians(toRawMotorRadians(RobotConstants.TURRET_MAX_ANGLE_RADIANS))
        );
        setAngleRadians(RobotConstants.TURRET_MIN_ANGLE_RADIANS);
        loopTimer.reset();
    }

    public void start() {
        turret.resetController();
        manualIntegral = 0.0;
        lastManualError = 0.0;
        loopTimer.reset();
    }

    public void limelightAim(LLResult result) {
        updateLoopState();
        if (controlMode != ControlMode.MANUAL_PID) {
            turret.resetController();
            manualIntegral = 0.0;
            lastManualError = 0.0;
            controlMode = ControlMode.MANUAL_PID;
        }

        if (result != null) {
            if (result.isValid()) {
                if (result.getTx() == 0) {
                    applyManualPositionPid(0);
                }
                else {
                    applyManualPositionPid(-result.getTx());
                }
            }
            else {
                applyManualPositionPid(0);
            }
        }
        else {
            applyManualPositionPid(0);
        }
    }
    public void manualControl(double input) {
        manualExtraPower = manualMaxPower * input;
    }
    public void loop() {
        updateLoopState();
        applyProfileConfigurables();
        if (controlMode != ControlMode.PROFILED) {
            turret.resetController();
            controlMode = ControlMode.PROFILED;
        }
        turret.update(loopState);
    }

    private void applyProfileConfigurables() {
        MotionProfilingCoefficients scaledCoefficients = MotorCoefficientScaler.fromLegacyTickSpace(
                new MotionProfilingCoefficients(kp, ki, kd, ks, kv, ka, maxVel, maxAcc, maxDec),
                encoderConverter
        );
        turretProfileCoefficients.setPidCoefficients(
                scaledCoefficients.getPidCoef().kp(),
                scaledCoefficients.getPidCoef().ki(),
                scaledCoefficients.getPidCoef().kd(),
                scaledCoefficients.getPidCoef().ks(),
                scaledCoefficients.getPidCoef().kv(),
                scaledCoefficients.getPidCoef().ka()
        );
        turretProfileCoefficients.setMotionProfileLimits(
                scaledCoefficients.getMaxVelocity(),
                scaledCoefficients.getMaxAcceleration(),
                scaledCoefficients.getMaxDeceleration()
        );
        turret.setTargetTolerance(Angle.fromRadians(
                mechanismDeltaToMotorRadians(Math.toRadians(targetToleranceDegrees))
        ));
        turret.setMaxPower(maxPower);
    }

    private void updateLoopState() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        double batteryVoltageFactor = 1.0 / getBatteryVoltage();
        if (dt <= 0.0 || dt > MAX_REASONABLE_LOOP_DT_SECONDS) {
            loopState.set(0.0, batteryVoltageFactor);
            return;
        }
        loopState.set(dt, batteryVoltageFactor);
    }

    private void applyManualPositionPid(double error) {
        double dt = loopState.getDt();
        if (dt <= 0) {
            return;
        }

        double derivative = (error - lastManualError) / dt;
        lastManualError = error;

        double output = rampPower(
                turretHardware.getPower(),
                kp * error + ki * manualIntegral + kd * derivative + ks * Math.signum(error),
                dt
        );
        int positionTicks = turretHardware.getCurrentPositionTicks();
        boolean motorTooLowPos = positionTicks < getMinAngleTicks();
        boolean motorTooHighPos = positionTicks > getMaxAngleTicks();
        if (Math.abs(error) < 1) {
            output = manualExtraPower;
        }
        if (output < maxPower) {
            manualIntegral += error;
        }
        if (motorTooLowPos && output < 0) {
            output = 0;
        }
        if (motorTooHighPos && output > 0) {
            output = 0;
        }
        turretHardware.maxPower(maxPower);
        turretHardware.setPower(output);
    }

    public TurretTelemetrySnapshot getTelemetrySnapshot(TelemetryMode mode, double nowSeconds) {
        Double currentAmps = mode.includes(TelemetryMode.DEBUG)
                ? currentSampler.get(nowSeconds, turret::getCurrentAmps)
                : null;
        int positionTicks = turretHardware.getCurrentPositionTicks();
        return new TurretTelemetrySnapshot(
                controlMode.name(),
                positionAimLutEnabled,
                lastAimPointWasVirtual,
                angleToGoal,
                targetMechanismAngleRadians,
                lastAimPointX,
                lastAimPointY,
                positionTicks,
                getMeasuredAngleRadians(),
                turretHardware.getVelocityTicksPerSecond(),
                turret.getPower(),
                encoderConverter.angleToTicks(turret.getReferenceState().getPosition()),
                encoderConverter.velocityToTicksPerSecond(turret.getReferenceState().getVelocity()),
                encoderConverter.accelerationToTicksPerSecondSquared(
                        turret.getReferenceState().getAcceleration()
                ),
                turretHardware.isOverCurrent(),
                positionTicks < getMinAngleTicks(),
                positionTicks > getMaxAngleTicks(),
                currentAmps
        );
    }

    @Override
    public void collectTelemetry(TelemetryCollector collector, TelemetryMode mode) {
        TurretTelemetrySnapshot snapshot = getTelemetrySnapshot(mode, collector.getNowSeconds());

        collector.add("turret", "over_current", snapshot.overCurrent,
                TelemetryMode.COMPETITION, TelemetryCostClass.BULK_CACHED);
        collector.add("turret", "target_rad", snapshot.targetAngleRadians,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("turret", "measured_rad", snapshot.measuredAngleRadians,
                TelemetryMode.DEBUG, TelemetryCostClass.BULK_CACHED);
        collector.add("turret", "position_ticks", snapshot.positionTicks,
                TelemetryMode.DEBUG, TelemetryCostClass.BULK_CACHED);
        collector.add("turret", "angle_to_goal_rad", snapshot.angleToGoalRadians,
                TelemetryMode.DEBUG, TelemetryCostClass.FORMATTED);
        collector.add("turret", "lut_enabled", snapshot.lutEnabled,
                TelemetryMode.DEBUG, TelemetryCostClass.STATIC);
        collector.add("turret", "lut_active", snapshot.usingVirtualAimPoint,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("turret", "aim_point_x", snapshot.aimPointX,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("turret", "aim_point_y", snapshot.aimPointY,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("turret", "velocity_tps", snapshot.measuredVelocityTicksPerSecond,
                TelemetryMode.DEBUG, TelemetryCostClass.BULK_CACHED);
        collector.add("turret", "power", snapshot.appliedPower,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("turret", "reference_position_ticks", snapshot.referencePositionTicks,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("turret", "reference_velocity_tps", snapshot.referenceVelocityTicksPerSecond,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("turret", "reference_accel_tps2",
                snapshot.referenceAccelerationTicksPerSecondSquared,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("turret", "at_lower_limit", snapshot.atLowerLimit,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("turret", "at_upper_limit", snapshot.atUpperLimit,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("turret", "current_amps", snapshot.currentAmps,
                TelemetryMode.DEBUG, TelemetryCostClass.NON_BULK);
    }

    private double getTargetPositionTicks() {
        return RobotConstants.TURRET_ZERO_OFFSET_TICKS
                + targetMechanismAngleRadians
                * RobotConstants.TURRET_MOTOR_TYPE.getTicksPerRadian()
                * RobotConstants.TURRET_EXTERNAL_GEAR_RATIO;
    }

    private Pose getAimTargetPose(Pose robotPose, boolean isRed) {
        Pose goalPose = isRed ? RED_GOAL_POSE : BLUE_GOAL_POSE;
        Pose virtualAimPose = null;
        if (positionAimLutEnabled) {
            PositionAimLut lut = isRed ? RED_POSITION_AIM_LUT : BLUE_POSITION_AIM_LUT;
            virtualAimPose = lut.getVirtualAimPoint(robotPose, positionAimLutNeighborCount);
        }

        Pose chosenPose = virtualAimPose != null ? virtualAimPose : goalPose;
        lastAimPointX = chosenPose.getX();
        lastAimPointY = chosenPose.getY();
        lastAimPointWasVirtual = virtualAimPose != null;
        return chosenPose;
    }

    private double computeTurretAngleRadians(Pose robotPose, Pose targetPose) {
        double fieldAngle = Math.atan2(
                targetPose.getY() - robotPose.getY(),
                targetPose.getX() - robotPose.getX()
        );
        return normalizeRadians(fieldAngle - robotPose.getHeading());
    }

    private double normalizeRadians(double angleRadians) {
        while (angleRadians > Math.PI) {
            angleRadians -= 2 * Math.PI;
        }
        while (angleRadians < -Math.PI) {
            angleRadians += 2 * Math.PI;
        }
        return angleRadians;
    }

    private double getMinAngleTicks() {
        return RobotConstants.TURRET_ZERO_OFFSET_TICKS
                + RobotConstants.TURRET_MIN_ANGLE_RADIANS
                * RobotConstants.TURRET_MOTOR_TYPE.getTicksPerRadian()
                * RobotConstants.TURRET_EXTERNAL_GEAR_RATIO;
    }

    private double getMaxAngleTicks() {
        return RobotConstants.TURRET_ZERO_OFFSET_TICKS
                + RobotConstants.TURRET_MAX_ANGLE_RADIANS
                * RobotConstants.TURRET_MOTOR_TYPE.getTicksPerRadian()
                * RobotConstants.TURRET_EXTERNAL_GEAR_RATIO;
    }

    private double toRawMotorRadians(double mechanismRadians) {
        return RobotConstants.TURRET_ZERO_OFFSET_MOTOR_RADIANS
                + mechanismRadians * RobotConstants.TURRET_EXTERNAL_GEAR_RATIO;
    }

    private double rawMotorRadiansToMechanismRadians(double rawMotorRadians) {
        return (rawMotorRadians - RobotConstants.TURRET_ZERO_OFFSET_MOTOR_RADIANS)
                / RobotConstants.TURRET_EXTERNAL_GEAR_RATIO;
    }

    private double mechanismDeltaToMotorRadians(double mechanismRadians) {
        return mechanismRadians * RobotConstants.TURRET_EXTERNAL_GEAR_RATIO;
    }

    private double rampPower(double current, double target, double dt) {
        double maxPowerChange = ramp * dt;
        double diff = target - current;
        if (Math.abs(diff) > maxPowerChange) {
            diff = Math.signum(diff) * maxPowerChange;
        }
        return current + diff;
    }

    private double getBatteryVoltage() {
        double minVoltage = Double.POSITIVE_INFINITY;
        for (com.qualcomm.robotcore.hardware.VoltageSensor sensor : hwmap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                minVoltage = Math.min(minVoltage, voltage);
            }
        }
        return minVoltage < Double.POSITIVE_INFINITY ? minVoltage : 12.0;
    }
}
