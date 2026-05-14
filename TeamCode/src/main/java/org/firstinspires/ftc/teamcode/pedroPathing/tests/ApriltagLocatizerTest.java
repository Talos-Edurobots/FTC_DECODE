package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "apriltag locatizer test", group = "tests")
@Disabled
public class ApriltagLocatizerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // april tag localization pipeline
        waitForStart();
        limelight.start(); // start
        while (opModeIsActive()) {
            LLResult llResult = limelight.getLatestResult();
            telemetry.addData("tag", llResult.getBotpose().getPosition());
            telemetry.update();
        }
    }
}
