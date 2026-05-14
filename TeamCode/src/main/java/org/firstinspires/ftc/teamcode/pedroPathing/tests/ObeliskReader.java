package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "obelisk reader")
@Disabled
public class ObeliskReader extends LinearOpMode {
    Limelight3A limelight;
    /*
    * Blue Goal: 20
    * Motif GPP: 21
    * Motif PGP: 22
    * Motif PPG: 23
    * Red Goal: 24
    */
    final int GPP = 21;
    final int PGP = 22;
    final int PPG = 23;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // obelisk reader pipeline
        waitForStart();
        limelight.start(); // start
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("motif", result.getFiducialResults().get(0).getFiducialId());
            }
            else {
                telemetry.addData("motif", "none");
            }
            telemetry.update();
        }
    }
}
