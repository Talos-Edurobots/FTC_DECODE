package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Hang;
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
import java.util.Timer;
import java.util.function.Supplier;
@Configurable
public class MainTeleOp implements TelemetryProvider {
    private static final int DRIVER_STATION_TELEMETRY_INTERVAL_MS = 200;

    public static int backVel = 1500;
    public static int frontVel = 1250;
    public static int turretOffset = 3;
    public static double hoodFarAngle = 0.1;
    public static double hoodCloseAngle = .1;
    public static boolean useShooterHoodLuts = true;
    public static double hoodLutTrim = 0.0;
    public static int hoodLutNeighborCount = HoodAngleLut.DEFAULT_NEIGHBOR_COUNT;
    public static boolean defaultUseLimelight = false;
    public static boolean useLimelightRelocalization = false;
    public static boolean logTelemetry = false;

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
    static boolean useTurret = true;
    private static final double VELOCITY_THRESHOLD = 0.1; // inches/sec
    private boolean isRobotNearlyStationary() {
        return follower.getVelocity().getMagnitude() < VELOCITY_THRESHOLD;
    }

    private HardwareManager hardwareManager;
    private Transfer transfer;
    private Shooter shooter;
    private IMU imu;
    private Turret turret;
    private Follower follower;
    private Timer pedroTimer;
    private Limelight3A limelight;
    private Leds leds;
    private DriveTrain drivetrain;
    private Pose3D llPose;
    private boolean automatedDrive = false;
    private boolean isFar = false;
    // Pedro Pathing lazy path generation for automated driving
    private Supplier<PathChain> autoPathChain;
    private static Pose BLUE_TARGET_POSE = new Pose(100, 37, Math.toRadians(-90));
    private Pose autoTargetPose;
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
    double botPoseX, botPoseY, botHeading, rawPoseX, rawPoseY, rawHeading;
    Hang hang;
    Pose teleOpStartPose;

    public void init(OpMode opMode, boolean isBlue) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.isBlue = isBlue;
        teleOpStartPose = RobotPoseStorage.hasPose() ? RobotPoseStorage.getPose() : startingPose;

        follower = (Follower) opMode.blackboard.get(RobotConstants.FOLLOWER_KEY);
        if (follower == null) {
            follower = PPConstants.createFollower(hardwareMap);
        }
        follower.setStartingPose(teleOpStartPose);
        follower.update();

        // Lazy path: generates a fresh BezierLine from the robot's current pose to the target each time
        autoTargetPose = isBlue ? BLUE_TARGET_POSE : BLUE_TARGET_POSE.mirror();
        autoPathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, autoTargetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, autoTargetPose.getHeading(), 0.8))
                .build();

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

        hang = new Hang();
        hang.init(hardwareMap);

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
        telemetryM.addData("start pose", teleOpStartPose);
        telemetryM.addLine("press a to reset");
        if (opMode.gamepad1.aWasPressed()) teleOpStartPose = startingPose;
        telemetryM.update(telemetry);
    }

    public void start() {
        limelight.pipelineSwitch(1);
        limelight.start();
        transfer.collect();
        transfer.update();
        turret.start();
        turret.setResetting(true);
        lastLoopTime = opMode.getRuntime();
        loopTimeStats.reset();
        automatedDrive = false;
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
            hang.setState(useHang);
        }

        if (transfer.getState() == Transfer.TransferState.SHOOT && transfer.isEmpty()) {
//            transfer.collect();
            leds.blink(Leds.Side.BOTH, 0.28, 3);
        } else if (transfer.getState() == Transfer.TransferState.SHOOT) {
            leds.setBoth(0.5);
        } else if (transfer.getState() == Transfer.TransferState.COLLECT) {
            leds.setBoth(0.33);
        } else if (transfer.getState() == Transfer.TransferState.STOP && shooter.isBusy()) {
            leds.pulse(Leds.Side.BOTH, 0.71, 0.3);
        } else if (transfer.getState() == Transfer.TransferState.STOP && !shooter.isBusy()) {
            leds.pulse(Leds.Side.BOTH, 0.5, .3);
        }
        if (useHang) {
            leds.rgb(dt);
        }
        leds.update(dt);

        turret.manualControl(opMode.gamepad1.left_trigger - opMode.gamepad1.right_trigger);
        if (turretFaceForwardOverride) {
            turret.faceForward();

        } else {
//            turret.lookToGoalWhileMoving(follower.getPose(), follower.getVelocity(), !isBlue);
            turret.lookToGoal(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading() - Math.toRadians(turretOffset)), !isBlue);

        }
        if (opMode.gamepad1.yWasPressed()) {
            turret.setResetting(true);
        }

        if (useTurret) turret.loop();
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

//        if (opMode.gamepad1.dpad_left) {
//            if (useShooterHoodLuts) {
//                hoodLutTrim -= dt * .8;
//            } else {
//                shooter.setHoodAngle(shooter.getHoodAngle() - dt * .8);
//            }
//        }
//        if (opMode.gamepad1.dpad_right) {
//            if (useShooterHoodLuts) {
//                hoodLutTrim += dt * .8;
//            } else {
//                shooter.setHoodAngle(shooter.getHoodAngle() + dt * .8);
//            }
//        }
        if (opMode.gamepad1.dpadRightWasPressed()) turretOffset++;
        if (opMode.gamepad1.dpadLeftWasPressed()) turretOffset--;
        if (transfer.getState() != Transfer.TransferState.COLLECT) {
            updateShooterAndHoodTargets(follower.getPose());
        } else {
            shooter.setIdle(true);
        }
        shooter.update();


        LLResult llResult;
//        double pinpointHeading = Math.toDegrees(follower.getHeading());
//        limelight.updateRobotOrientation(pinpointHeading);
//        if (follower.getVelocity().getMagnitude() < 0.1) {
//            llResult = limelight.getLatestResult();
//            if (llResult.isValid()) {
//                llPose = llResult.getBotpose_MT2();
//                double PoseX = llPose.getPosition().x * 39.3701;
//                double PoseY = llPose.getPosition().y * 39.3701;
//                follower.setPose(new Pose(PoseX, PoseY, follower.getPose().getHeading()));
//            }
//        }

        if (follower.getVelocity().getMagnitude() < 0.1) {
            llResult = limelight.getLatestResult();
            if (llResult.isValid()) {
                rawPoseX = llResult.getBotpose().getPosition().x;
                rawPoseY = llResult.getBotpose().getPosition().y;
                rawHeading = llResult.getBotpose().getOrientation().getYaw();
                botPoseX = (rawPoseX * 39.3701) + 72;
                botPoseY = (rawPoseY * 39.3701) + 72;
                botHeading = Math.toRadians(rawHeading);
            }
        }

        // Limelight pose update using Megatag2 (filtered)
        if (useLimelightRelocalization && isRobotNearlyStationary()) {
            llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D mt2Pose = llResult.getBotpose_MT2();
                double poseX = (mt2Pose.getPosition().x * 39.3701) + 72;
                double poseY = (mt2Pose.getPosition().y * 39.3701) + 72;
//                double heading = Math.toRadians(mt2Pose.getOrientation().getYaw());
                follower.setPose(new Pose(poseX, poseY, follower.getPose().getHeading()));
                telemetryM.addLine("Limelight MT2 relocalized");
            }
        }

        // D‑pad down: update heading with Megatag1
        if (useLimelightRelocalization && opMode.gamepad1.dpadDownWasPressed() && isRobotNearlyStationary()) {
            llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D mt1Pose = llResult.getBotpose();
                double headingDeg = mt1Pose.getOrientation().getYaw();
                Pose currentPose = follower.getPose();
                follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(headingDeg)));
                telemetryM.addLine("Limelight MT1 heading updated");
            }
        }

        if (opMode.gamepad2.x) {
            follower.holdPoint(follower.getPose());
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
        double forward = opMode.gamepad1.left_stick_y * mult * (isBlue?1:-1);
        double strafe = -opMode.gamepad1.left_stick_x * mult * (isBlue?1:-1);
        double rotate = opMode.gamepad1.right_stick_x * mult;
        double heading = follower.getPose().getHeading();
        lastHeadingRadians = heading;
        if (Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate) > 0) automatedDrive = false;

        // gamepad2 dpad up: start automated path to target position
        boolean dpadUp = opMode.gamepad2.dpadUpWasPressed();
        if (dpadUp && !automatedDrive) {
            follower.followPath(autoPathChain.get(), false);
            automatedDrive = true;
        }
        else if (dpadUp) {
            follower.breakFollowing();
            follower.startTeleopDrive();
        }

        // Cancel automated drive when path finishes
        if (automatedDrive && !follower.isBusy()) {
            automatedDrive = false;
        }

        if (!automatedDrive) {
            // Manual control via Drivetrain subsystem
            drivetrain.FieldCentricAccelerationDrive(strafe, forward, rotate, heading, mult, dt);
//            follower.setTeleOpDrive(forward, strafe, rotate, false, heading);

        }
        // follower.update() always runs so pose estimation stays current
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
        shooter.setIdle(false);
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
        collector.add("system", "raw_limelight x", rawPoseX, TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "raw_limelight y", rawPoseY, TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "raw_limelight heading", rawHeading, TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "limelight_pose", llPose, TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "limelight x", botPoseX, TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "limelight_y", botPoseY, TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "limelight_radians", botHeading, TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "transfer empty", transfer.isEmpty(), TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
        collector.add("system", "transfer full", transfer.isFull(), TelemetryMode.COMPETITION, TelemetryCostClass.CHEAP);
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
