package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "test intake with drivetrain", group = "testing")
public class IntakeTestWithDriveTrain extends LinearOpMode {
    public DcMotor intake, leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.dcMotor.get("intake");
        imu = hardwareMap.get(IMU.class, "imu");
        leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        leftBackDrive = hardwareMap.dcMotor.get("leftBack");
        rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
        rightBackDrive = hardwareMap.dcMotor.get("rightBack");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        waitForStart();
        while (opModeIsActive()) {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            telemetry.addData("intake power", intake.getPower());
            telemetry.addData("right back power", rightBackDrive.getPower());
            telemetry.addData("right back current", ((DcMotorEx)rightBackDrive).getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is "start" on Xbox-style controllers.
            // press options button to reset IMU
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            // we multiply denominator variable by a variable named "straferSpeedFactor" with a value greater than 1
            // in order to reduce the strafing speed. The normal strafing speed is to high and thus difficult to control the robot
            double speed = .5;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = speed*((rotY + rotX + rx) / denominator);
            double backLeftPower = speed*((rotY - rotX + rx) / denominator);
            double frontRightPower = speed*((rotY - rotX - rx) / denominator);
            double backRightPower = speed*(rotY + rotX - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);
        }
    }
}
