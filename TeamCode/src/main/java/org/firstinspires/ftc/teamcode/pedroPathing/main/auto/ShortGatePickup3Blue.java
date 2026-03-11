package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "ShortGatePickup3Blue")
public class ShortGatePickup3Blue extends OpMode {
    ShortGatePickup3 auto = new ShortGatePickup3();
    @Override
    public void init() {
        auto.init(hardwareMap, telemetry, true);
    }

    @Override
    public void loop() {
        auto.loop();
    }
}
