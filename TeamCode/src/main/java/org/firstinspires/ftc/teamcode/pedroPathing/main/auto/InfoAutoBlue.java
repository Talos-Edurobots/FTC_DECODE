package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Info New Blue")
public class InfoAutoBlue extends OpMode {
    InfoAuto auto = new InfoAuto();
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
