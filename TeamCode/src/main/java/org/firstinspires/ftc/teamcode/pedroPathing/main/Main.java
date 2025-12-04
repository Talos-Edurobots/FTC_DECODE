package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem.Pinpoint;


//Im here
@TeleOp(name = "Main TeleOp", group = "main")
@Configurable
public class Main extends LinearOpMode {
    DriveTrain driveTrain;
    Intake intake;
    IMU imu;
    Pinpoint pinpoint;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    Pose2D robotPos;

    @Override
    public void runOpMode() throws InterruptedException {
        double oldTime = 0, newTime, dt;

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

            pinpoint.update();
            robotPos = pinpoint.getPosition();
            Drawing.drawRobot(new Pose(robotPos.getX( DistanceUnit.INCH), robotPos.getY(DistanceUnit.INCH), robotPos.getHeading(AngleUnit.RADIANS)));

            if (gamepad1.options) {
                imu.resetYaw();
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

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            double heading = robotPos.getHeading(AngleUnit.RADIANS);

            double speed = 1;
            driveTrain.FieldCentricAccelerationDrive(strafe, forward, rotate, heading, speed, dt);

            telemetryM.addData("Heading", heading);
            telemetryM.addData("dx", dt);
            telemetryM.addData("intake velocity", intake.getIntakeMotor().getVelocity());
            telemetryM.addData("intake current", intake.getIntakeMotor().getCurrent(CurrentUnit.AMPS));
            telemetryM.addData("pinpoint pos", robotPos);
            telemetryM.update(telemetry);

            sleep(20);

        }
    }
}
