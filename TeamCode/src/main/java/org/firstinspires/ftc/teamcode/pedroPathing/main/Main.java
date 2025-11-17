package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.main.drivetrain.DriveTrain;


//Im here
@TeleOp(name = "Main TeleOp", group = "main")
@Configurable
public class Main extends LinearOpMode {
    DriveTrain driveTrain;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        double oldTime = 0, newTime, dt;
        driveTrain = new DriveTrain(hardwareMap);
        driveTrain.init();
        DcMotor intake = hardwareMap.dcMotor.get(RobotConstants.INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(RobotConstants.IMU_PARAMETERS);
        imu.resetYaw();
        waitForStart();
        while (opModeIsActive()){
            newTime = getRuntime();
            dt = newTime - oldTime;
            oldTime = newTime;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double speed = 1;
            driveTrain.FieldCentricAccelerationDrive(strafe, forward, rotate, heading, speed, dt);

            telemetry.addData("Heading", heading);
            telemetry.addData("dx", dt);
            telemetry.update();

            sleep(20);

        }
    }
}
