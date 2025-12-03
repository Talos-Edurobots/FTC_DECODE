package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "test drawing", group = "testing")
public class testDrawing extends LinearOpMode {
    private final FieldManager fieldManager = PanelsField.INSTANCE.getField();
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            fieldManager.circle(6);
            fieldManager.update();
        }
    }
}
