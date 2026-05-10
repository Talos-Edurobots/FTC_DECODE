package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "AutoV2Blue")
public class AutoV2Blue extends OpMode {
    AutoV2 auto = new AutoV2();

    @Override
    public void init() {
        auto.init(hardwareMap, telemetry, true);
    }

    @Override
    public void loop() {
        auto.loop();
    }
}
