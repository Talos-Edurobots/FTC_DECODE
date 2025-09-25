package org.firstinspires.ftc.teamcode.pedroPathing.resourses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ftccontrol.panels.Panels;
import org.ftccontrol.panels.graph.Graph;

@TeleOp(name = "Panels Graph Example", group = "Examples")
public class PanelsGraphExample extends LinearOpMode {

    // Create a graph object
    private Graph myGraph;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Panels (must be done before using it)
        Panels.init(telemetry);

        // Register a graph with a title and unit
        myGraph = Panels.graph("Motor Power", "time (s)", "power");

        waitForStart();

        double t0 = getRuntime();

        while (opModeIsActive()) {
            double elapsed = getRuntime() - t0;
            double motorPower = Math.sin(elapsed);  // some dummy data

            // Add data point (x = time, y = value)
            myGraph.add(elapsed, motorPower);

            // Panels updates alongside telemetry
            Panels.update();

            idle();
        }
    }
}
