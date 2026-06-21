package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Back New Red")
public class NewBackAutoRed extends OpMode {
    NewBackAuto auto = new NewBackAuto();
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
