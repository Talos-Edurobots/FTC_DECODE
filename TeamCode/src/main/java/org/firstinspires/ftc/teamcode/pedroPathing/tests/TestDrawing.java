package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "test drawing", group = "testing")
public class TestDrawing extends LinearOpMode {
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final Style style = new Style(
            "", "#3F51B5", .75
    );
    private static final double ROBOT_RADIUS = 9; // in inches
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            double x = (Math.random() * 200) - 100;          // Replace with real odometry
            double y = (Math.random() * 200) - 100;          // Replace with real odometry
            double heading = Math.random() * 360;    // Replace with IMU or OTOS
//            fieldManager.setStyle(style);
//            fieldManager.moveCursor(0, 0);
//            fieldManager.circle(60);
//
//            fieldManager.update();
            telemetryM.addLine("running");
            telemetryM.update(telemetry);
            drawRobot(new Pose(x, y, heading), style);
            panelsField.update();
            sleep(500);
            idle();
        }
    }
}
