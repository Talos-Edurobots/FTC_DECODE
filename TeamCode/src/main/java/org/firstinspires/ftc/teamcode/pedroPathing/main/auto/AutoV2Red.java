package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Auto2Red")
public class AutoV2Red extends OpMode {
    AutoV2 auto = new AutoV2();

    @Override
    public void init() {
        auto.init(hardwareMap, telemetry, false);
    }

    @Override
    public void loop() {
        auto.loop();
    }

    @Override
    public void stop(){ auto.stop();}
}
