package org.firstinspires.ftc.teamcode.pedroPathing.main.auto.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "blue solo short auto")
public class SoloShortAutoBlue extends OpMode {
    SoloShortAuto opMode = new SoloShortAuto();
    @Override
    public void init() {
        opMode.init(hardwareMap, telemetry, true);
    }

    @Override
    public void loop() {
        opMode.loop();
    }
}
