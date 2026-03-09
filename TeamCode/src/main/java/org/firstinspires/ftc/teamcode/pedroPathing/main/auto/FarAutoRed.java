package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
