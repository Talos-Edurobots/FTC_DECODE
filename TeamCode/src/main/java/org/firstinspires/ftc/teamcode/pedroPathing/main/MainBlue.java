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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.ColorSensors;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Gate;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Hang;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Leds;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Pinpoint;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Drawing;

import java.util.function.Supplier;


//Im here
@TeleOp(name = "MainBlue TeleOp", group = "!main")
@Configurable
public class MainBlue extends LinearOpMode {
    static int backVel = 1500;
    static int frontVel = 1300;
    HardwareManager hardwareManager;
    DriveTrain driveTrain;
    ColorSensors colors;
    Intake intake;
    Shooter shooter;
    Gate gate = new Gate();
    IMU imu;
    Turret turret;
    Pinpoint pinpoint;
    Follower follower;
    Hang hang;
    private Limelight3A limelight;
    Supplier<PathChain> pathChain;
    Pose startingPose = new Pose(72, 72, Math.toRadians(180));
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    Leds leds;
    boolean automatedDrive = false;
    boolean isFar = false;
    double turretError;
    int pipeline = 3;
    boolean slowMode = false;
    boolean useLimelight = false;
    boolean useHang = false;
    boolean activateStop = false;
    static double hoodFarAngle = 0.1, hoodCloseAngle = .3;

    @Override
    public void runOpMode() throws InterruptedException {
        double oldTime = 0, newTime, dt;

        follower = (Follower) blackboard.get(RobotConstants.FOLLOWER_KEY);
        if (follower == null) follower = PPConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();
        hardwareManager = new HardwareManager(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(2);

        colors = new ColorSensors();
        colors.init(hardwareMap);

        leds = new Leds();
        leds.init(hardwareMap);
        hang = new Hang();
        hang.init(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.init();
        shooter.run(true);
        Shooter.targetVelocity = frontVel;
        shooter.setHoodAngle(hoodCloseAngle);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(RobotConstants.IMU_PARAMETERS);

        driveTrain = new DriveTrain(hardwareMap);
        driveTrain.init();

        driveTrain = new DriveTrain(hardwareMap);
        driveTrain.init();

        intake = new Intake(hardwareMap);
        intake.init();

        turret = new Turret(hardwareMap);
        turret.init();

//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }

        limelight.start();
        while(opModeInInit()) {
            telemetryM.addLine("running alliance " + (pipeline==2 ? "blue":"red"));
            telemetryM.addLine("press back to change");
            telemetryM.addData("pipeline", pipeline);
            if (gamepad1.backWasPressed()) {
                pipeline = pipeline == 2 ? 3 : 2;
                limelight.pipelineSwitch(pipeline);
            }
            telemetryM.update(telemetry);
        }
        waitForStart();
        follower.startTeleopDrive();
        gate.init(hardwareMap);
        gate.activate();
        intake.setCurrentState(Intake.IntakeState.INTAKE);

        while (opModeIsActive()){
            newTime = getRuntime();
            dt = newTime - oldTime;
            oldTime = newTime;
            hardwareManager.update();
            follower.update();
            if (gamepad1.backWasPressed()) {
                if (!useHang) {
                    intake.setCurrentState(Intake.IntakeState.STOP);
                    shooter.run(false);
                }
                useHang ^= true;
            };
            hang.update(1, useHang?90:0);
            LLResult result = limelight.getLatestResult();
            boolean isTurretTarget = Math.abs(result.getTx())<3 && result.getTx() != 0;
            double color1 = shooter.isBusy() ? .28 : isTurretTarget ? .5 : .333;
            double color2 = isFar ? .555 : .722;
            if (isTurretTarget && shooter.isBusy()) {
                leds.blinkLeft(0.2, dt, 1, 0);
            }
            else {
                leds.setLeft(color1);
            }
            if (intake.getCurrentState() == Intake.IntakeState.STOP) {
                leds.blinkRight(0.2, dt, color2, 1);
            }
            else {
                leds.setRight(color2);
            }
            turret.manualControl(gamepad1.left_trigger - gamepad1.right_trigger);
            if (useLimelight) turret.limelightAim(result);
            else {
//                turret.lookToGoal(follower.getPose(), false);
                turret.lookToGoalWhileMoving(follower.getPose(), follower.getVelocity(), false);
                turret.loop();
            }
            MotorConfig.setDt(dt);
            if (gamepad1.dpadDownWasPressed()) useLimelight ^= true;
            if (gamepad1.xWasPressed()) {
                isFar ^= true;
                Shooter.targetVelocity = isFar ? backVel : frontVel;
                shooter.setHoodAngle(isFar ? hoodFarAngle : hoodCloseAngle);
            }

            if (gamepad1.options) {
                imu.resetYaw();
                follower.setPose(follower.getPose().setHeading(0));
            }
            // Shooter control
            if (gamepad1.dpadUpWasPressed()) {
                shooter.changeState();
            }
            shooter.update();

            if (gamepad1.dpadLeftWasPressed()) {
                shooter.setHoodAngle(shooter.getHoodAngle() - .1);
            }
            if (gamepad1.dpadRightWasPressed()) {
                shooter.setHoodAngle(shooter.getHoodAngle() + .1);
            }
            if (gamepad1.startWasPressed()) {
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(180)));
            }

//            flickers.leftFlick(gamepad1.left_bumper);
//            flickers.rightFlick(activateStop);

//            pinpoint.update();
//            robotPos = pinpoint.getPosition();
            Drawing.drawRobot(follower.getPose(), turret.getAngleToGoal());
            Drawing.sendPacket();


            if (gamepad2.x) {
                follower.activateAllPIDFs();
            } else if (gamepad2.a) {
                follower.deactivateAllPIDFs();
            }
//            if (gamepad1.dpadDownWasPressed()) {
////                follower.followPath(pathChain.get());
//                automatedDrive = true;
//            }
//            if (automatedDrive && (gamepad1.xWasPressed() || !follower.isBusy())) {
////                follower.startTeleopDrive();
//                automatedDrive = false;
//            }

//            boolean aPressed = gamepad1.aWasPressed();
//            if (aPressed && intake.getCurrentState() == Intake.IntakeState.INTAKE) {
//                intake.setCurrentState(Intake.IntakeState.STOP);
//            }
//            else if (aPressed && intake.getCurrentState() == Intake.IntakeState.STOP) {
//                intake.setCurrentState(Intake.IntakeState.INTAKE);
//            }
//            else if (gamepad1.yWasPressed()) {
//                intake.setCurrentState(Intake.IntakeState.OUTTAKE);
//            }
//            else if (gamepad1.bWasPressed()) {
//                intake.setCurrentState(Intake.IntakeState.STOP);
//            }
            colors.update();
            boolean isFull = colors.isFull();
            boolean rightBumb = gamepad1.rightBumperWasPressed();
            if ((intake.getCurrentState() == Intake.IntakeState.INTAKE && rightBumb) || isFull) {
                intake.setCurrentState(Intake.IntakeState.STOP);
                gate.activate();
            } else if (intake.getCurrentState() == Intake.IntakeState.STOP && rightBumb) {
                intake.setCurrentState(Intake.IntakeState.INTAKE);
            }
            boolean leftBump = gamepad1.leftBumperWasPressed();
            if (leftBump) {
                intake.setCurrentState(Intake.IntakeState.INTAKE);
                gate.deactivate();
            }
            intake.update();

            if (gamepad1.leftStickButtonWasPressed() || gamepad1.rightStickButtonWasPressed()) slowMode ^= true;
            double mult = slowMode ? 0.25 : 1;
            double forward = gamepad1.left_stick_y * mult;
            double strafe = gamepad1.left_stick_x * mult;
            double rotate = -gamepad1.right_stick_x * mult;
//            double heading = follower.getHeading();
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double speed = 1;
//            driveTrain.FieldCentricAccelerationDrive(strafe, forward, rotate, heading, speed, dt);
            if (!automatedDrive) {
                follower.setTeleOpDrive(forward, strafe, rotate, false);
            }
            follower.update();

            telemetryM.addData("fps", 1/ dt);
            telemetryM.addData("color 1", colors.is1Detected());
            telemetryM.addData("color 2", colors.is2Detected());
            telemetryM.addData("color 3", colors.is3Detected());
            telemetryM.addData("is all", colors.isFull());
            telemetryM.addData("timer", colors.getFullTIme());
            telemetryM.addData("intake status", intake.getCurrentState());
            telemetryM.addData("intake current", intake.getCurrent());
            telemetryM.addData("shooter vel", shooter.getVelocity());
            telemetryM.addData("shooter current", shooter.getCurrent());
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
        if (isStopRequested()) {
            blackboard.put(RobotConstants.FOLLOWER_KEY, null);
            blackboard.put(RobotConstants.ALLIANCE_KEY, null);
        }
    }
}
