package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.HoodAngleLut;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Leds;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.ShooterHoodLuts;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryCollector;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryCostClass;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryHub;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.LoopTimeStats;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryMode;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.TelemetryProvider;
import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.ThrottledValue;

import java.util.HashMap;

public class MainTeleOp implements TelemetryProvider {
    private static final int DRIVER_STATION_TELEMETRY_INTERVAL_MS = 100;

    public static int backVel = 1500;
    public static int frontVel = 1250;
    public static double hoodFarAngle = 0.1;
    public static double hoodCloseAngle = .1;
    public static boolean useShooterHoodLuts = true;
    public static double hoodLutTrim = 0.0;
    public static int hoodLutNeighborCount = HoodAngleLut.DEFAULT_NEIGHBOR_COUNT;
    public static boolean defaultUseLimelight = false;

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final TelemetryHub telemetryHub = new TelemetryHub();
    private final LoopTimeStats loopTimeStats = new LoopTimeStats();
    private final ThrottledValue<Double> intakeCurrentSampler = new ThrottledValue<>(0.1);
    private final ThrottledValue<Double> totalCurrentSampler = new ThrottledValue<>(0.2);
    private final Pose startingPose = new Pose(72, 72, Math.toRadians(180));

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private OpMode opMode;
    private boolean isBlue;

    private HardwareManager hardwareManager;
    private Transfer transfer;
    private Shooter shooter;
    private IMU imu;
    private Turret turret;
    private Follower follower;
    private Limelight3A limelight;
    private Leds leds;
    private DriveTrain drivetrain;
    private boolean automatedDrive = false;
    private boolean isFar = false;
    static boolean slowMode = false;
    private boolean useLimelight = defaultUseLimelight;
    static boolean turretFaceForwardOverride = false;
    private boolean useHang = false;
    static boolean shooting = false;
    private double lastLoopTime = 0.0;
    private double lastLoopDt = 0.0;
    private double lastHeadingRadians = 0.0;
    private double lastVisionTx = 0.0;
    private boolean lastTurretTargetLock = false;
    private double lastShooterDistanceFromGoal = 0.0;
    private double lastShooterTargetVelocity = 0.0;
    private double lastHoodTargetPosition = 0.0;

    public void init(OpMode opMode, boolean isBlue) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.isBlue = isBlue;

        Pose teleOpStartPose = RobotPoseStorage.hasPose() ? RobotPoseStorage.getPose() : startingPose;

        follower = (Follower) opMode.blackboard.get(RobotConstants.FOLLOWER_KEY);
        if (follower == null) {
            follower = PPConstants.createFollower(hardwareMap);
        }
        follower.setStartingPose(teleOpStartPose);
        follower.update();

        hardwareManager = new HardwareManager(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(DRIVER_STATION_TELEMETRY_INTERVAL_MS);
        limelight.pipelineSwitch(isBlue ? 2 : 3);

        leds = new Leds();
        leds.init(hardwareMap);

        shooter = new Shooter(hardwareMap);
        shooter.init();
        shooter.run(true);
        updateShooterAndHoodTargets(teleOpStartPose);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(RobotConstants.IMU_PARAMETERS);

        transfer = new Transfer(hardwareMap);
        transfer.init(hardwareMap);

        turret = new Turret(hardwareMap);
        turret.init();

        drivetrain = new DriveTrain(hardwareMap);
        drivetrain.init();

        useLimelight = defaultUseLimelight;
        automatedDrive = false;
        isFar = false;
        slowMode = false;
        useHang = false;
        shooting = false;
        turretFaceForwardOverride = false;
        lastLoopTime = 0.0;
        lastLoopDt = 0.0;
        lastHeadingRadians = 0.0;
        lastVisionTx = 0.0;
        lastTurretTargetLock = false;
        lastShooterDistanceFromGoal = 0.0;
        lastShooterTargetVelocity = Shooter.targetVelocity;
        lastHoodTargetPosition = shooter.getHoodAngle();
        loopTimeStats.reset();

        telemetryHub.clearProviders();
        telemetryHub.setMode(TelemetryMode.COMPETITION);
        telemetryHub.register(this);
        telemetryHub.register(shooter);
        telemetryHub.register(turret);
    }

    public void init_loop() {
        int pipeline = limelight.getStatus().getPipelineIndex();
        telemetryM.addLine("running alliance " + (pipeline == 2 ? "blue" : "red"));
        telemetryM.addLine("press back to change");
        telemetryM.addData("pipeline", pipeline);
        if (opMode.gamepad1.backWasPressed()) {
            limelight.pipelineSwitch(pipeline == 2 ? 3 : 2);
        }
        telemetryM.update(telemetry);
    }

    public void start() {
        limelight.start();
//        follower.startTeleopDrive(true);
        transfer.collect();
        transfer.update();
        turret.start();
        lastLoopTime = opMode.getRuntime();
        loopTimeStats.reset();
    }

    public void loop() {
        double newTime = opMode.getRuntime();
        double dt = newTime - lastLoopTime;
        lastLoopTime = newTime;
        lastLoopDt = dt;
        boolean resetLoopStats = opMode.gamepad2.startWasPressed();
        if (resetLoopStats) {
            loopTimeStats.reset();
        } else {
            loopTimeStats.record(dt);
        }

        hardwareManager.update();

        if (opMode.gamepad2.backWasPressed()) {
            telemetryHub.cycleMode();
        }
        if (opMode.gamepad2.yWasPressed()) {
            turretFaceForwardOverride ^= true;
        }

        if (opMode.gamepad1.backWasPressed()) {
            if (!useHang) {
                transfer.stop();
                transfer.update();
                shooter.run(false);
            }
            useHang ^= true;
        }

        LLResult result = limelight.getLatestResult();
        boolean isTurretTarget = result != null && result.getTx() != 0 && Math.abs(result.getTx()) < 3;
        lastTurretTargetLock = isTurretTarget;
        lastVisionTx = result == null ? 0.0 : result.getTx();
        double color1 = shooter.isBusy() ? .28 : isTurretTarget ? .5 : .333;
        double color2 = isFar ? .555 : .722;
        if (isTurretTarget && shooter.isBusy()) {
            leds.blinkLeft(0.2, dt, 1, 0);
        } else {
            leds.setLeft(color1);
        }
        if (transfer.getState() == Transfer.TransferState.STOP) {
            leds.blinkRight(0.2, dt, color2, 1);
        } else {
            leds.setRight(color2);
        }

        turret.manualControl(opMode.gamepad1.left_trigger - opMode.gamepad1.right_trigger);
        if (turretFaceForwardOverride) {
            turret.faceForward();
            turret.loop();
        } else if (useLimelight) {
            turret.limelightAim(result);
        } else {
//            turret.lookToGoalWhileMoving(follower.getPose(), follower.getVelocity(), !isBlue);
            turret.lookToGoal(follower.getPose(), !isBlue);
            turret.loop();
        }

        if (opMode.gamepad1.dpadDownWasPressed()) {
            useLimelight ^= true;
        }
        if (opMode.gamepad1.xWasPressed()) {
            isFar ^= true;
            if (!useShooterHoodLuts) {
                Shooter.targetVelocity = isFar ? backVel : frontVel;
                shooter.setHoodAngle(isFar ? hoodFarAngle : hoodCloseAngle);
            }
        }

//        if (opMode.gamepad1.options) {
//            imu.resetYaw();
//            follower.setPose(follower.getPose().setHeading(0));
//        }

        if (opMode.gamepad1.dpadUpWasPressed()) {
            shooter.changeState();
        }

        if (opMode.gamepad1.dpad_left) {
            if (useShooterHoodLuts) {
                hoodLutTrim -= dt * .8;
            } else {
                shooter.setHoodAngle(shooter.getHoodAngle() - dt * .8);
            }
        }
        if (opMode.gamepad1.dpad_right) {
            if (useShooterHoodLuts) {
                hoodLutTrim += dt * .8;
            } else {
                shooter.setHoodAngle(shooter.getHoodAngle() + dt * .8);
            }
        }
        updateShooterAndHoodTargets(follower.getPose());
        shooter.update();
//        if (opMode.gamepad1.startWasPressed()) {
//            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(180)));
//        }

//        Drawing.drawRobot(follower.getPose(), turret.getAngleToGoal());
//        Drawing.sendPacket();

        if (opMode.gamepad2.x) {
            follower.activateAllPIDFs();
        } else if (opMode.gamepad2.a) {
            follower.deactivateAllPIDFs();
        }

        shooting = opMode.gamepad1.left_bumper;
        if (shooting) {
            transfer.shoot();
        } else if (opMode.gamepad1.leftBumperWasPressed()) {
            transfer.collect();
        } else if (opMode.gamepad1.rightBumperWasPressed()) {
            if (transfer.getState() == Transfer.TransferState.COLLECT) {
                transfer.stop();
            } else {
                transfer.collect();
            }
        }
        if (!shooting && transfer.getState() == Transfer.TransferState.SHOOT) {
            transfer.stop();
        }
        transfer.update();

        if (opMode.gamepad1.leftStickButtonWasPressed() || opMode.gamepad1.rightStickButtonWasPressed()) {
            slowMode ^= true;
        }
        double mult = slowMode ? 0.25 : 1;
        double forward = -opMode.gamepad1.left_stick_y * mult;
        double strafe = opMode.gamepad1.left_stick_x * mult;
        double rotate = opMode.gamepad1.right_stick_x * mult;
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastHeadingRadians = heading;

        if (!automatedDrive) {
            drivetrain.FieldCentricAccelerationDrive(strafe, forward, rotate, heading, mult, dt);
        }
        follower.update();
        telemetryHub.publish(telemetryM, telemetry, newTime);
    }

    private void updateShooterAndHoodTargets(Pose robotPose) {
        if (robotPose == null) {
            return;
        }

        if (!useShooterHoodLuts) {
            lastShooterDistanceFromGoal = ShooterHoodLuts.distanceToGoal(robotPose, !isBlue);
            lastShooterTargetVelocity = Shooter.targetVelocity;
            lastHoodTargetPosition = shooter.getHoodAngle();
            return;
        }

        double distanceFromGoal = ShooterHoodLuts.distanceToGoal(robotPose, !isBlue);
        double targetVelocity = ShooterHoodLuts.SHOOTER_VELOCITY_LUT.getTargetVelocity(
                distanceFromGoal
        );
        if (Double.isNaN(targetVelocity) || Double.isInfinite(targetVelocity)) {
            targetVelocity = isFar ? backVel : frontVel;
        }

        hoodLutNeighborCount = Math.max(1, hoodLutNeighborCount);
        double hoodPosition = ShooterHoodLuts.HOOD_ANGLE_LUT.getHoodPosition(
                distanceFromGoal,
                shooter.getVelocity(),
                hoodLutNeighborCount
        );
        if (Double.isNaN(hoodPosition) || Double.isInfinite(hoodPosition)) {
            hoodPosition = isFar ? hoodFarAngle : hoodCloseAngle;
        }
        hoodPosition += hoodLutTrim;

        Shooter.setTargetVelocity(targetVelocity);
        shooter.setHoodAngle(hoodPosition);
        lastShooterDistanceFromGoal = distanceFromGoal;
        lastShooterTargetVelocity = targetVelocity;
        lastHoodTargetPosition = shooter.getHoodAngle();
    }

    public void stop(HashMap blackboard) {
        blackboard.put(RobotConstants.FOLLOWER_KEY, null);
        blackboard.put(RobotConstants.ALLIANCE_KEY, null);
    }

    @Override
    public void collectTelemetry(TelemetryCollector collector, TelemetryMode mode) {
        LoopTimeStats.Snapshot loopStats = loopTimeStats.snapshot();

//        collector.add("system", "telemetry_mode", mode, TelemetryMode.COMPETITION,
//                TelemetryCostClass.CHEAP);
//        collector.add("system", "avg fps", loopStats.averageMillis, TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
//        collector.add("system", "min fps", loopStats.worstMillis, TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
//        collector.add("system", "1% lows", loopStats.p99Millis, TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
//        collector.add("system", ".1% lows", loopStats.p999Millis, TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("system", "loop_hz", lastLoopDt > 0 ? 1 / lastLoopDt : 0.0,
                TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "loop_avg_fps", 1000/loopStats.averageMillis,
                TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "loop_worst_fps", 1000/loopStats.worstMillis,
                TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "loop_1pct_low_fps", loopStats.onePercentLowHertz(),
                TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "loop_0_1pct_low_fps", loopStats.pointOnePercentLowHertz(),
                TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "loop_stats_samples", loopStats.sampleCount,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("system", "dt", lastLoopDt, TelemetryMode.DEBUG,
                TelemetryCostClass.CHEAP);

        collector.add("drive", "heading_rad", lastHeadingRadians, TelemetryMode.COMPETITION,
                TelemetryCostClass.BULK_CACHED);
        collector.add("drive", "pose", follower.getPose(), TelemetryMode.DEBUG,
                TelemetryCostClass.FORMATTED);
        collector.add("drive", "x", follower.getPose().getX(), TelemetryMode.DEBUG,
                TelemetryCostClass.CHEAP);
        collector.add("drive", "robot_heading", follower.getHeading(), TelemetryMode.DEBUG,
                TelemetryCostClass.CHEAP);

        collector.add("vision", "tx", lastVisionTx, TelemetryMode.DEBUG,
                TelemetryCostClass.CHEAP);

        collector.add("intake", "over_current", transfer.isOverCurrent()?.9:0, TelemetryMode.COMPETITION,
                TelemetryCostClass.BULK_CACHED);
        collector.add("intake", "shooting", shooting, TelemetryMode.COMPETITION,
                TelemetryCostClass.CHEAP);
        collector.add("intake", "color_1_detected", transfer.is1Detected()?1:0, TelemetryMode.DEBUG,
                TelemetryCostClass.CHEAP);
        collector.add("intake", "color_2_detected", transfer.is2Detected()?1.1:0, TelemetryMode.DEBUG,
                TelemetryCostClass.CHEAP);
        collector.add("intake", "color_3_detected", transfer.is3Detected()?1.2:0, TelemetryMode.DEBUG,
                TelemetryCostClass.CHEAP);
        collector.add("intake", "full_time", transfer.getFullTime(), TelemetryMode.DEBUG,
                TelemetryCostClass.CHEAP);
        collector.add("intake", "current_amps",
                mode.includes(TelemetryMode.DEBUG)
                        ? intakeCurrentSampler.get(collector.getNowSeconds(), transfer::getCurrent)
                        : null,
                TelemetryMode.DEBUG, TelemetryCostClass.NON_BULK);

        collector.add("robot", "shot_preset", isFar ? "far" : "close",
                TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("robot", "shooter_hood_luts_enabled", useShooterHoodLuts,
                TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("robot", "shooter_distance_from_goal", lastShooterDistanceFromGoal,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("robot", "shooter_lut_target_tps", lastShooterTargetVelocity,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("robot", "hood_lut_target", lastHoodTargetPosition,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("robot", "hood_lut_trim", hoodLutTrim,
                TelemetryMode.DEBUG, TelemetryCostClass.CHEAP);
        collector.add("robot", "slow_mode", slowMode, TelemetryMode.COMPETITION,
                TelemetryCostClass.CHEAP);
        collector.add("robot", "turret_forward_override", turretFaceForwardOverride,
                TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("robot", "hang_mode", useHang, TelemetryMode.COMPETITION,
                TelemetryCostClass.CHEAP);
        collector.add("robot", "total_current_amps",
                mode.includes(TelemetryMode.DEBUG)
                        ? totalCurrentSampler.get(
                                collector.getNowSeconds(),
                                hardwareManager::getTotalCurrentDrawAmps
                        )
                        : null,
                TelemetryMode.DEBUG, TelemetryCostClass.NON_BULK);
    }
}
