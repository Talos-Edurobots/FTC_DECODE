package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "test drawing", group = "testing")
public class testDrawing extends LinearOpMode {
    private final FieldManager fieldManager = PanelsField.INSTANCE.getField();
    private final Style style = new Style(
            "", "#3F51B5", .75
    );
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            fieldManager.setStyle(style);
            fieldManager.moveCursor(0, 0);
            fieldManager.circle(60);
            fieldManager.update();
        }
    }
}
