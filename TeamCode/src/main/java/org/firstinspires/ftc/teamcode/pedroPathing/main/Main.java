package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Draw;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Flickers;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Pinpoint;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Shooter;


//Im here
@TeleOp(name = "Main TeleOp", group = "main")
@Configurable
public class Main extends LinearOpMode {
    DriveTrain driveTrain;
    Intake intake;
    Shooter shooter;
    Flickers flickers;
    IMU imu;
    Pinpoint pinpoint;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    Pose robotPos;

    @Override
    public void runOpMode() throws InterruptedException {
        double oldTime = 0, newTime, dt;

        shooter = new Shooter(hardwareMap);
        shooter.init();

        flickers = new Flickers(hardwareMap);
        flickers.init();

        pinpoint = new Pinpoint(hardwareMap);
        pinpoint.init();

        driveTrain = new DriveTrain(hardwareMap);
        driveTrain.init();

        intake = new Intake(hardwareMap);
        intake.init();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(RobotConstants.IMU_PARAMETERS);
        imu.resetYaw();

        waitForStart();
        while (opModeIsActive()){
            newTime = getRuntime();
            dt = newTime - oldTime;
            oldTime = newTime;

            // Shooter control
            if (gamepad1.dpadUpWasPressed()) {
                shooter.changeRun();
            }
            shooter.update(2500, dt);

            if (gamepad1.dpadLeftWasPressed()) {
                shooter.setHoodAngle(shooter.getHoodAngle() - .1);
            }
            if (gamepad1.dpadRightWasPressed()) {
                shooter.setHoodAngle(shooter.getHoodAngle() + .1);
            }
            flickers.leftFlick(gamepad1.left_bumper);
            flickers.rightFlick(gamepad1.right_bumper);

            pinpoint.update();
            robotPos = pinpoint.getPosition();
            Draw.drawRobot(robotPos, RobotConstants.ROBOT_DRAW_STYLE);
            Draw.update();


            if (gamepad1.options) {
                imu.resetYaw();
                pinpoint.getPinpoint().setHeading(0, AngleUnit.DEGREES);
            }

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
            double heading = robotPos.getHeading();

            double speed = 1;
            driveTrain.FieldCentricAccelerationDrive(strafe, forward, rotate, heading, speed, dt);

            telemetryM.addData("shooter vel", shooter.getVelocity());
            telemetryM.addData("shooter current", shooter.getCurrent());
            telemetryM.addData("Heading", heading);
            telemetryM.addData("dt", dt);
            telemetryM.addData("intake velocity", intake.getIntakeMotor().getVelocity());
            telemetryM.addData("intake current", intake.getIntakeMotor().getCurrent(CurrentUnit.AMPS));
            telemetryM.addData("pinpoint pos", robotPos);
            telemetryM.update(telemetry);

            sleep(20);

        }
    }
}
