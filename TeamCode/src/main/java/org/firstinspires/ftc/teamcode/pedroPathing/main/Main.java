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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.main.config.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.main.config.PPConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.config.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Flickers;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Leds;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Pinpoint;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Turret;

import java.util.List;
import java.util.function.Supplier;


//Im here
@TeleOp(name = "Main TeleOp", group = "main")
@Configurable
public class Main extends LinearOpMode {
    HardwareManager hardwareManager;
    DriveTrain driveTrain;
    Intake intake;
    Shooter shooter;
    Flickers flickers;
    IMU imu;
    Turret turret;
    Pinpoint pinpoint;
    Follower follower;
    private Limelight3A limelight;
    Supplier<PathChain> pathChain;
    Pose startingPose = new Pose(45, 98);
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    Leds leds;
    boolean automatedDrive = false;
    boolean far = true;
    double turretError;

    @Override
    public void runOpMode() throws InterruptedException {
        double oldTime = 0, newTime, dt;

        follower = PPConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();
        hardwareManager = new HardwareManager(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        leds = new Leds();
        leds.init(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.init();
        shooter.run(false);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(RobotConstants.IMU_PARAMETERS);

        driveTrain = new DriveTrain(hardwareMap);
        driveTrain.init();

        flickers = new Flickers();
        flickers.init(hardwareMap);

        pinpoint = new Pinpoint(hardwareMap);
        pinpoint.init();

        driveTrain = new DriveTrain(hardwareMap);
        driveTrain.init();

        intake = new Intake(hardwareMap);
        intake.init();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        leds.setLeft(.72);
        leds.setRight(.72);
        waitForStart();
        while (opModeIsActive()){

            newTime = getRuntime();
            dt = newTime - oldTime;
            oldTime = newTime;
            hardwareManager.update();
            follower.update();

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    turretError = result.getTx();
                }
            }
            if (gamepad1.xWasPressed()) {
                far ^= true;
                Shooter.targetVelocity = far ? 2400 : 1200;
            }

            if (gamepad1.options) {
                imu.resetYaw();
            }
            // Shooter control
            if (gamepad1.dpadUpWasPressed()) {
                shooter.changeRun();
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
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();


            if (gamepad2.options) {
                follower.activateAllPIDFs();
            } else if (gamepad2.share) {
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

            if (gamepad1.aWasPressed()) {
                intake.setCurrentState(Intake.IntakeState.INTAKE);
            }
            else if (gamepad1.bWasPressed()) {
                intake.setCurrentState(Intake.IntakeState.STOP);
            }
            else if (gamepad1.yWasPressed()) {
                intake.setCurrentState(Intake.IntakeState.OUTTAKE);
            }
            intake.update();

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
//            double heading = follower.getHeading();
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double speed = 1;
            driveTrain.FieldCentricAccelerationDrive(strafe, forward, rotate, heading, speed, dt);
            if (!automatedDrive) {
//                follower.setTeleOpDrive(forward, strafe, rotate, false);
            }
//            follower.update();

            telemetryM.addData("fps", 1/ dt);
            telemetryM.addData("shooter vel", shooter.getVelocity());
            telemetryM.addData("shooter current", shooter.getCurrent());
            telemetryM.addData("shooter target", shooter.getTargetVelocity());
            telemetryM.addData("Heading", heading);
            telemetryM.addData("dt", dt);
            telemetryM.addData("intake velocity", intake.getVelocity());
            telemetryM.addData("intake current", intake.getCurrent());
//            telemetryM.addData("pinpoint pos", follower.getPose());
            telemetryM.update(telemetry);
        }
    }
}
