package org.firstinspires.ftc.teamcode.pedroPathing.main.auto.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "far auto red")
public class FarAutoRed extends OpMode {
    FarAuto auto = new FarAuto();
    @Override
    public void init() {
        auto.init(hardwareMap, telemetry, false);
    }

    @Override
    public void loop() {
        auto.loop();
    }
}
