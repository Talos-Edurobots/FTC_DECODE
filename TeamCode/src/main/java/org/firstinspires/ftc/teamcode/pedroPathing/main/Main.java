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
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Hang;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Flickers;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Leds;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Pinpoint;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Drawing;

import java.util.function.Supplier;


//Im here
@TeleOp(name = "Main TeleOp", group = "main")
@Configurable
public class Main extends LinearOpMode {
    static int backVel = 2000;
    static int frontVel = 1200;
    HardwareManager hardwareManager;
    DriveTrain driveTrain;
    Intake intake;
    Shooter shooter;
    Flickers flickers;
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
    boolean far = true;
    double turretError;
    int pipeline = 3;
    boolean slowMode = false;
    boolean useLimelight = false;

    @Override
    public void runOpMode() throws InterruptedException {
        double oldTime = 0, newTime, dt;

        follower = PPConstants.createFollower(hardwareMap);
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

        leds = new Leds();
        leds.init(hardwareMap);
        hang = new Hang();
        hang.init(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.init();
        shooter.run(false);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(RobotConstants.IMU_PARAMETERS);

        driveTrain = new DriveTrain(hardwareMap);
        driveTrain.init();

        flickers = new Flickers();
        flickers.init(hardwareMap);

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
        while (opModeIsActive()){
            newTime = getRuntime();
            dt = newTime - oldTime;
            oldTime = newTime;
            hardwareManager.update();
            MotorConfig.setDt(dt);
            double color = shooter.isBusy() ? .28 : .5;
            leds.setLeft(color);
            leds.setRight(color);
            follower.update();
            hang.update(1, 0);
            LLResult result = limelight.getLatestResult();
            turret.manualControl(gamepad1.left_trigger - gamepad1.right_trigger);
            if (useLimelight) turret.limelightAim(result);
            else {
                turret.lookToGoal(follower.getPose(), false);
                turret.loop();
            }
            if (gamepad1.dpadDownWasPressed()) useLimelight ^= true;
            if (gamepad1.xWasPressed()) {
                far ^= true;
                Shooter.targetVelocity = far ? backVel : frontVel;
            }

            if (gamepad1.options) {
                imu.resetYaw();
                follower.setPose(follower.getPose().setHeading(0));
            }
            // Shooter control
            if (gamepad1.dpadUpWasPressed()) {
                shooter.changeState();
            }
            shooter.update(dt);

            if (gamepad1.dpadLeftWasPressed()) {
                shooter.setHoodAngle(shooter.getHoodAngle() - .1);
            }
            if (gamepad1.dpadRightWasPressed()) {
                shooter.setHoodAngle(shooter.getHoodAngle() + .1);
            }

            flickers.leftFlick(gamepad1.left_bumper);
            flickers.rightFlick(gamepad1.right_bumper);

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

            boolean aPressed = gamepad1.aWasPressed();
            if (aPressed && intake.getCurrentState() == Intake.IntakeState.INTAKE) {
                intake.setCurrentState(Intake.IntakeState.STOP);
            }
            else if (aPressed && intake.getCurrentState() == Intake.IntakeState.STOP) {
                intake.setCurrentState(Intake.IntakeState.INTAKE);
            }
            else if (gamepad1.yWasPressed()) {
                intake.setCurrentState(Intake.IntakeState.OUTTAKE);
            }
            else if (gamepad1.bWasPressed()) {
                intake.setCurrentState(Intake.IntakeState.STOP);
            }
            intake.update();

            slowMode = gamepad1.left_stick_button || gamepad1.right_stick_button;
            double mult = slowMode ? 0.4 : 1;
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
            telemetryM.addData("intake status", intake.getCurrentState());
            telemetryM.addData("shooter vel", shooter.getVelocity());
            telemetryM.addData("shooter current", shooter.getCurrent());
            telemetryM.addData("shooter target", shooter.getTargetVelocity());
            telemetryM.addData("shooter filtered vel", shooter.filteredVelocity);
            telemetryM.addData("is impact detected", shooter.isImpactDetected());
            telemetryM.addData("Heading", heading);
            telemetryM.addData("dt", dt);
            telemetryM.addData("intake velocity", intake.getVelocity());
            telemetryM.addData("intake current", intake.getCurrent());
            telemetryM.addData("pinpoint pos", follower.getPose());
            telemetryM.addData("follower x", follower.getPose().getX());
            telemetryM.addData("pinpoint heading", follower.getHeading());
            telemetryM.update(telemetry);
        }
    }
}
