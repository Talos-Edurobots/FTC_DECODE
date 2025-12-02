package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.util.Log;

@TeleOp(name = "LogcatTest676767", group = "tests")
public  class LogcatTestFTCDecode6767 extends LinearOpMode {

    // Unique logcat tag (filter on this!)
    private static final String TAG = "FTCLOG";
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetryM.addLine("Ready. Start Logcat and press PLAY67.");
        telemetryM.update();

        waitForStart();

        double lastTime = getRuntime();

        while (opModeIsActive()) {

            sleep(500); // Adjust loop delay as needed
            // Example variables
            double x = Math.random() * 100;          // Replace with real odometry
            double y = Math.random() * 100;          // Replace with real odometry
            double heading = Math.random() * 360;    // Replace with IMU or OTOS

            double now = getRuntime();
            double loopMs = (now - lastTime);
            lastTime = now;

            // 🔥 LOGCAT OUTPUT (this is what you filter on)
            Log.d(TAG,
                    "x=" + x +
                            ", y=" + y +
                            ", heading=" + heading +
                            ", loopMs=" + loopMs
            );

            // Optional telemetry to DS (VERY slow)
            telemetryM.addData("Loop", loopMs);
            telemetryM.update(telemetry);

        }
    }
}