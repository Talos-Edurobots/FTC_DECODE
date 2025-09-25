package org.firstinspires.ftc.teamcode.pedroPathing.resourses;

import com.bylazar.graph.GraphManager;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;


@TeleOp(name = "Panels Graph Example", group = "Examples")
public class PanelsTest extends LinearOpMode {

    private GraphManager graphManager;
    private TelemetryManager telemetryA;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetryA = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryA.debug("Initialized Telemetry Managernafjsdjf");
        telemetryA.update();


        waitForStart();
        double t0 = getRuntime();

        while (opModeIsActive()) {
            double elapsed = getRuntime() - t0;
            double value = Math.sin(elapsed);

            // Add point to graph


            // Update Panels


            idle();
        }
    }
}
