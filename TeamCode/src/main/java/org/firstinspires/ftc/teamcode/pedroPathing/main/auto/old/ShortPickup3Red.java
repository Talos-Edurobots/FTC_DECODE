package org.firstinspires.ftc.teamcode.pedroPathing.main.auto.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "ShortPickup3Red")
public class ShortPickup3Red extends OpMode {
    ShortPickup3 auto = new ShortPickup3();
    @Override
    public void init() {
        auto.init(hardwareMap, telemetry, false);
    }

    @Override
    public void loop() {
        auto.loop();
    }
}
