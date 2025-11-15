package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "right back drive test", group = "testin")
@Disabled
public class RightBackDrive extends LinearOpMode {
    DcMotor rightBackDrive;
    @Override
    public void runOpMode() throws InterruptedException {
        rightBackDrive = hardwareMap.dcMotor.get("rightBack");
        waitForStart();
        while (opModeIsActive()) {
            rightBackDrive.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("right back power", rightBackDrive.getPower());
            telemetry.addData("right back current", ((DcMotorEx)rightBackDrive).getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
