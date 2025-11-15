package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Graph", group = "testing")
public class TestGraph extends LinearOpMode {
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ElapsedTime timer = new ElapsedTime();


    private double sinVariable = 0.0;
    private double cosVariable = 0.0;
    private double constVariable = 0.0;
    private double dampedSine = 0.0;
    private double lissajous = 0.0;
    private double ramp = 0.0;
    private double squareWave = 0.0;

    @Override
    public void runOpMode() {
        updateSignals();
        waitForStart();
        while (opModeIsActive()) {
            updateSignals();
            sleep(20);
        }
    }


    private void updateSignals() {
        double t = timer.seconds();

        sinVariable = Math.sin(t);
        cosVariable = Math.cos(t);
        constVariable = 1.0;

        dampedSine = Math.exp(-0.2 * t) * Math.sin(2 * t);

        lissajous = Math.sin(3 * t + Math.PI / 2) * Math.cos(2 * t);

        ramp = (t % 5.0) / 5.0;

        squareWave = Math.sin(2 * Math.PI * 0.5 * t) > 0 ? 1.0 : -1.0;

        panelsTelemetry.addData("sin", sinVariable);
        panelsTelemetry.addData("cos", cosVariable);
        panelsTelemetry.addData("dampedSine", dampedSine);
        panelsTelemetry.addData("lissajous", lissajous);
        panelsTelemetry.addData("ramp", ramp);
        panelsTelemetry.addData("square", squareWave);
        panelsTelemetry.addData("const", constVariable);



        panelsTelemetry.addLine("extra1:" + t + " extra2:" + (t * t) + " extra3:" + Math.sqrt(t));

        panelsTelemetry.update(telemetry);
    }
}