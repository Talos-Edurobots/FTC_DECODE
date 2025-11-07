package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.main.drivetrain.DriveTrain;


public class Main extends LinearOpMode {
    DriveTrain driveTrain;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        driveTrain.init();
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(RobotConstants.IMU_PARAMETERS);
        waitForStart();
        while (opModeIsActive()) {
            driveTrain.fieldCentricDrive(
                    -gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS),
                     1
            );
        }
    }
}
