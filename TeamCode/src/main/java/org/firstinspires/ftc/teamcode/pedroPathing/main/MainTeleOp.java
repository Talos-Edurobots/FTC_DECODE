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
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.ColorSensors;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Gate;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Leds;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Turret;

import java.util.HashMap;

public class MainTeleOp {
    public static int backVel = 1500;
    public static int frontVel = 1300;
    public static double hoodFarAngle = 0.1;
    public static double hoodCloseAngle = .1;
    public static boolean defaultUseLimelight = false;

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final Pose startingPose = new Pose(72, 72, Math.toRadians(180));

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private OpMode opMode;
    private boolean isBlue;

    private HardwareManager hardwareManager;
    private Intake intake;
    private Shooter shooter;
    private IMU imu;
    private Turret turret;
    private Follower follower;
    private Limelight3A limelight;
    private Leds leds;
    private Gate gate;
    private ColorSensors colors;

    private boolean automatedDrive = false;
    private boolean isFar = false;
    private boolean slowMode = false;
    private boolean useLimelight = defaultUseLimelight;
    private boolean useHang = false;
    private boolean shooting = false;
    private double lastLoopTime = 0.0;

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
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(isBlue ? 2 : 3);

        leds = new Leds();
        leds.init(hardwareMap);

        shooter = new Shooter(hardwareMap);
        shooter.init();
        shooter.run(true);
        Shooter.targetVelocity = frontVel;
        shooter.setHoodAngle(hoodCloseAngle);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(RobotConstants.IMU_PARAMETERS);

        intake = new Intake(hardwareMap);
        intake.init();

        turret = new Turret(hardwareMap);
        turret.init();

        colors = new ColorSensors();
        colors.init(hardwareMap);

        gate = new Gate();
        gate.init(hardwareMap);

        useLimelight = defaultUseLimelight;
        automatedDrive = false;
        isFar = false;
        slowMode = false;
        useHang = false;
        shooting = false;
        lastLoopTime = 0.0;
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
        follower.startTeleopDrive(true);
        gate.activate();
        intake.setCurrentState(Intake.IntakeState.INTAKE);
        turret.start();
        lastLoopTime = opMode.getRuntime();
    }

    public void loop() {
        double newTime = opMode.getRuntime();
        double dt = newTime - lastLoopTime;
        lastLoopTime = newTime;

        hardwareManager.update();

        if (opMode.gamepad1.backWasPressed()) {
            if (!useHang) {
                intake.setCurrentState(Intake.IntakeState.STOP);
                shooter.run(false);
            }
            useHang ^= true;
        }

        LLResult result = limelight.getLatestResult();
        boolean isTurretTarget = result != null && result.getTx() != 0 && Math.abs(result.getTx()) < 3;
        double color1 = shooter.isBusy() ? .28 : isTurretTarget ? .5 : .333;
        double color2 = isFar ? .555 : .722;
        if (isTurretTarget && shooter.isBusy()) {
            leds.blinkLeft(0.2, dt, 1, 0);
        } else {
            leds.setLeft(color1);
        }
        if (intake.getCurrentState() == Intake.IntakeState.STOP) {
            leds.blinkRight(0.2, dt, color2, 1);
        } else {
            leds.setRight(color2);
        }

        turret.manualControl(opMode.gamepad1.left_trigger - opMode.gamepad1.right_trigger);
        if (useLimelight) {
            turret.limelightAim(result);
        } else {
            turret.lookToGoalWhileMoving(follower.getPose(), follower.getVelocity(), !isBlue);
            turret.loop();
        }

        if (opMode.gamepad1.dpadDownWasPressed()) {
            useLimelight ^= true;
        }
        if (opMode.gamepad1.xWasPressed()) {
            isFar ^= true;
            Shooter.targetVelocity = isFar ? backVel : frontVel;
            shooter.setHoodAngle(isFar ? hoodFarAngle : hoodCloseAngle);
        }

        if (opMode.gamepad1.options) {
            imu.resetYaw();
            follower.setPose(follower.getPose().setHeading(0));
        }

        if (opMode.gamepad1.dpadUpWasPressed()) {
            shooter.changeState();
        }
        shooter.update();

        if (opMode.gamepad1.dpadLeftWasPressed()) {
            shooter.setHoodAngle(shooter.getHoodAngle() - .1);
        }
        if (opMode.gamepad1.dpadRightWasPressed()) {
            shooter.setHoodAngle(shooter.getHoodAngle() + .1);
        }
        if (opMode.gamepad1.startWasPressed()) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(180)));
        }

        Drawing.drawRobot(follower.getPose(), turret.getAngleToGoal());
        Drawing.sendPacket();

        if (opMode.gamepad2.x) {
            follower.activateAllPIDFs();
        } else if (opMode.gamepad2.a) {
            follower.deactivateAllPIDFs();
        }

        colors.update();
        boolean isFull = colors.isFull();
        shooting = opMode.gamepad1.left_bumper;
        boolean rightBumper = opMode.gamepad1.rightBumperWasPressed();
        if ((intake.getCurrentState() == Intake.IntakeState.INTAKE && rightBumper) || (isFull && !shooting)) {
            intake.setCurrentState(Intake.IntakeState.STOP);
            gate.activate();
        } else if (intake.getCurrentState() == Intake.IntakeState.STOP && rightBumper) {
            intake.setCurrentState(Intake.IntakeState.INTAKE);
        }
        if (opMode.gamepad1.leftBumperWasPressed()) {
            intake.setCurrentState(Intake.IntakeState.INTAKE);
            gate.deactivate();
        }
        intake.update();

        if (opMode.gamepad1.leftStickButtonWasPressed() || opMode.gamepad1.rightStickButtonWasPressed()) {
            slowMode ^= true;
        }
        double mult = slowMode ? 0.25 : 1;
        double forward = opMode.gamepad1.left_stick_y * mult;
        double strafe = opMode.gamepad1.left_stick_x * mult;
        double rotate = -opMode.gamepad1.right_stick_x * mult;
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (!automatedDrive) {
            follower.setTeleOpDrive(forward, strafe, rotate, false);
        }
        follower.update();

        telemetryM.addData("fps", dt > 0 ? 1 / dt : 0);
        telemetryM.addData("color 1", colors.is1Detected());
        telemetryM.addData("color 2", colors.is2Detected());
        telemetryM.addData("color 3", colors.is3Detected());
        telemetryM.addData("is all", colors.isFull());
        telemetryM.addData("timer", colors.getFullTIme());
        telemetryM.addData("isShooting", shooting);
        telemetryM.addData("intake status", intake.getCurrentState());
        telemetryM.addData("intake current", intake.getCurrent());
        telemetryM.addData("total current", hardwareManager.getTotalCurrentDrawAmps());
        telemetryM.addData("shooter vel", shooter.getVelocity());
        telemetryM.addData("shooter current", shooter.getCurrent1());
        telemetryM.addData("shooter target", shooter.getTargetVelocity());
        telemetryM.addData("shooter shooter running", shooter.getRun());
        telemetryM.addData("shooter filtered vel", shooter.filteredVelocity);
        telemetryM.addData("is impact detected", shooter.isImpactDetected());
        telemetryM.addData("Heading", heading);
        telemetryM.addData("dt", dt);
        telemetryM.addData("pinpoint pos", follower.getPose());
        telemetryM.addData("follower x", follower.getPose().getX());
        telemetryM.addData("pinpoint heading", follower.getHeading());
        telemetryM.update(telemetry);
    }

    public void stop(HashMap blackboard) {
        blackboard.put(RobotConstants.FOLLOWER_KEY, null);
        blackboard.put(RobotConstants.ALLIANCE_KEY, null);
    }
}
