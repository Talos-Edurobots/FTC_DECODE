package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "New Blue")
public class NewAutoBlue extends OpMode {
    NewAuto auto = new NewAuto();
    @Override
    public void init() {
        auto.init(hardwareMap, telemetry, true);
    }

    @Override
    public void loop() {
        auto.loop();
    }
}
