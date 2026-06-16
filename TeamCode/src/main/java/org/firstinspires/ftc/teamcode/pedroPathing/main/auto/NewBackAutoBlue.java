package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Back New Blue")
public class NewBackAutoBlue extends OpMode {
    NewBackAuto auto = new NewBackAuto();
    @Override
    public void init() {
        auto.init(hardwareMap, telemetry, true);
    }

    @Override
    public void loop() {
        auto.loop();
    }

    @Override
    public void stop(){ auto.stop();}
}
