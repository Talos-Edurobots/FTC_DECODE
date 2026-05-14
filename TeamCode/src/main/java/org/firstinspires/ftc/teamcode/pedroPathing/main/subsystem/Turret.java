package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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

@Configurable
public class Turret {
    private static final double MAX_REASONABLE_LOOP_DT_SECONDS = 0.25;

    private enum ControlMode {
        PROFILED,
        MANUAL_PID
    }

    static double maxPower = RobotConstants.TURRET_CONFIG.maxPower;
    static double kp = RobotConstants.TURRET_CONFIG.kP;
    static double ki = RobotConstants.TURRET_CONFIG.kI;
    static double kd = RobotConstants.TURRET_CONFIG.kD;
    static double ks = RobotConstants.TURRET_CONFIG.kS;
    static double kv = RobotConstants.TURRET_CONFIG.kV;
    static double ka = RobotConstants.TURRET_CONFIG.kA;
    static double maxVel = RobotConstants.TURRET_CONFIG.maxVelocity;
    static double maxAcc = RobotConstants.TURRET_CONFIG.maxAcceleration;
    static double maxDec = RobotConstants.TURRET_CONFIG.maxDeceleration;
    static double manualMaxPower = .2, ramp = 1;
    public static double movingShotLeadFactor = 0.01;
    private final HardwareMap hwmap;
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
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

    private ControlMode controlMode = ControlMode.PROFILED;
    private double angleToGoal;
    private double targetMechanismAngleRadians = 0.0;
    private double manualExtraPower = 0.0;
    private double manualIntegral = 0.0;
    private double lastManualError = 0.0;

    public Turret(HardwareMap hwmap) {
        this.hwmap = hwmap;
        turretHardware.hwName(RobotConstants.TURRET_MOTOR_NAME);
        turretHardware.direction(RobotConstants.TURRET_MOTOR_DIRECTION);
        turretHardware.zeroPowerBehavior(RobotConstants.TURRET_ZERO_POWER_BEHAVIOR);
        turretHardware.maxPower(RobotConstants.TURRET_LIMITS.getMaxPower());
        turretHardware.currentAlert(RobotConstants.TURRET_LIMITS.getCurrentAlertAmps());
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
        turret.setTargetAngle(Angle.fromRadians(toMotorRadians(targetMechanismAngleRadians)));
        if (controlMode != ControlMode.PROFILED) {
            turret.resetController();
            controlMode = ControlMode.PROFILED;
        }
    }

    public double getAngleToGoal(){
        return angleToGoal;
    }

    public void lookToGoal(Pose pose, boolean isRed) {
        double atan2;
        if (isRed) {
            atan2 = Math.atan2(RED_GOAL_POSE.getY()-pose.getY(), RED_GOAL_POSE.getX()-pose.getX());
        } else {
            atan2 = Math.atan2(BLUE_GOAL_POSE.getY()-pose.getY(), BLUE_GOAL_POSE.getX()-pose.getX());
        }
        double rad = (atan2 - pose.getHeading());
        rad = (rad > Math.PI) ? rad - 2 * Math.PI : rad;
        angleToGoal = rad;
        setAngleRadians(angleToGoal);
        telemetryM.addData("angle to goal", angleToGoal);
        telemetryM.addData("rad", rad);
        telemetryM.addData("atan2", rad);
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
                Angle.fromRadians(toMotorRadians(RobotConstants.TURRET_MIN_ANGLE_RADIANS)),
                Angle.fromRadians(toMotorRadians(RobotConstants.TURRET_MAX_ANGLE_RADIANS))
        );
        faceForward();
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
                telemetryM.addLine("valid result");
                if (result.getTx() == 0) {
                    applyManualPositionPid(0);
                    telemetryM.addLine("0 power");
                }
                else {
                    applyManualPositionPid(-result.getTx());
                    telemetryM.addLine("running turret");
                }
            }
            else {
                applyManualPositionPid(0);
                telemetryM.addLine("invalid result");
            }
        }
        else {
            applyManualPositionPid(0);
            telemetryM.addLine("null result");
        }
        telemetryM.addData("kp", kp);
        telemetryM.addData("kd", kd);
        telemetryM.addData("tx", result == null ? 0 : result.getTx());
        telemetryM.addData("max power", maxPower);
        telemetryM.addData("power", turret.getPower());
    }
    public void manualControl(double input) {
        manualExtraPower = manualMaxPower * input;
        telemetryM.addData("turret extra power", manualExtraPower);
    }
    public void loop() {
        updateLoopState();
        applyProfileConfigurables();
        if (controlMode != ControlMode.PROFILED) {
            turret.resetController();
            controlMode = ControlMode.PROFILED;
        }
        turret.update(loopState);
        telemetryM.addData("turret mode", controlMode);
        telemetryM.addData("power", turret.getPower());
        telemetryM.addData("velocity", turretHardware.getVelocityTicksPerSecond());
        telemetryM.addData(
                "ref vel",
                encoderConverter.velocityToTicksPerSecond(turret.getReferenceState().getVelocity())
        );
        telemetryM.addData("position", turretHardware.getCurrentPositionTicks());
        telemetryM.addData("min pos", getMinAngleTicks());
        telemetryM.addData("max pos", getMaxAngleTicks());
        telemetryM.addData(
                "ref pos",
                encoderConverter.angleToTicks(turret.getReferenceState().getPosition())
        );
        telemetryM.addData(
                "ref a",
                encoderConverter.accelerationToTicksPerSecondSquared(
                        turret.getReferenceState().getAcceleration()
                )
        );
        telemetryM.addData("current", turret.getCurrentAmps());
        telemetryM.addData("kp profile", kp);
        telemetryM.addData("ki profile", ki);
        telemetryM.addData("kd profile", kd);
        telemetryM.addData("ks profile", ks);
        telemetryM.addData("kv profile", kv);
        telemetryM.addData("ka profile", ka);
        telemetryM.addData("max vel profile", maxVel);
        telemetryM.addData("max acc profile", maxAcc);
        telemetryM.addData("max dec profile", maxDec);
        telemetryM.addData("target", getTargetPositionTicks());
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
            telemetryM.addLine("turret motor pos too low");
        }
        if (motorTooHighPos && output > 0) {
            output = 0;
            telemetryM.addLine("turret motor pos too high");
        }
        telemetryM.addData("output", output);
        telemetryM.addData("extra power 2", manualExtraPower);
        telemetryM.addData("error", error);
        telemetryM.addData("pos", positionTicks);
        telemetryM.addData("too low", motorTooLowPos);
        telemetryM.addData("too high", motorTooHighPos);
        turretHardware.maxPower(maxPower);
        turretHardware.setPower(output);
    }

    private double getTargetPositionTicks() {
        return targetMechanismAngleRadians
                * RobotConstants.TURRET_MOTOR_TYPE.getTicksPerRadian()
                * RobotConstants.TURRET_EXTERNAL_GEAR_RATIO;
    }

    private double getMinAngleTicks() {
        return RobotConstants.TURRET_MIN_ANGLE_RADIANS
                * RobotConstants.TURRET_MOTOR_TYPE.getTicksPerRadian()
                * RobotConstants.TURRET_EXTERNAL_GEAR_RATIO;
    }

    private double getMaxAngleTicks() {
        return RobotConstants.TURRET_MAX_ANGLE_RADIANS
                * RobotConstants.TURRET_MOTOR_TYPE.getTicksPerRadian()
                * RobotConstants.TURRET_EXTERNAL_GEAR_RATIO;
    }

    private double toMotorRadians(double mechanismRadians) {
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
