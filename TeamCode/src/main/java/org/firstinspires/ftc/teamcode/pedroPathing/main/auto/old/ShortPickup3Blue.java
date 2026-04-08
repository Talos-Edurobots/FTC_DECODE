package org.firstinspires.ftc.teamcode.pedroPathing.main.auto.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "ShortPickup3Blue")
public class ShortPickup3Blue extends OpMode {
    ShortPickup3 auto = new ShortPickup3();
    @Override
    public void init() {
        auto.init(hardwareMap, telemetry, true);
    }

    @Override
    public void loop() {
        auto.loop();
    }
}
