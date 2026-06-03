package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
//@Configurable
@Disabled
public class TestUpdateConfigurables extends LinearOpMode {
    static int testInt = 0;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    @Override
    public void runOpMode() throws InterruptedException{
        while (opModeIsActive()) {
            sleep(500);
            testInt++;
            PanelsConfigurables.INSTANCE.refreshClass(this);
            telemetryM.addData("testInt", testInt);
            telemetryM.update();
        }
    }
}

