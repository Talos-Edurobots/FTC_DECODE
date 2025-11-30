package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//todo check if PanelsTelemetry is working on the driver station

@TeleOp(name = "panelsTelemetry", group = "testing")
public class TestPanelsTelemetry extends LinearOpMode {
    TelemetryManager pTel = PanelsTelemetry.INSTANCE.getTelemetry();
    @Override
    public void runOpMode() {
        double lastTime = getRuntime();
        pTel.addLine("hello world");
        pTel.update();
        waitForStart();
        while (opModeIsActive()) {
            double dt = getRuntime() - lastTime;
            double cos = Math.cos(getRuntime());
            pTel.addData("cosine of t", cos);
            lastTime = getRuntime();
            pTel.update(telemetry);
        }
    }
}
