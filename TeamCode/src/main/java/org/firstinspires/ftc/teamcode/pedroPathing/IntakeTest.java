package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "test intake", group = "testing")
public class IntakeTest extends LinearOpMode {
    DcMotor intake;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.dcMotor.get("intake");
        waitForStart();
        while (opModeIsActive()) {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            telemetry.addData("intake power", intake.getPower());
            telemetry.update();
        }
    }
}
