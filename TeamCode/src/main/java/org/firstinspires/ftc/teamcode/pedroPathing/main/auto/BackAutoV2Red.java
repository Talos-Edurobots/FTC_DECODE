package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "BackAutoV2Red")
public class BackAutoV2Red extends OpMode {
    BackAutoV2 auto = new BackAutoV2();
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
