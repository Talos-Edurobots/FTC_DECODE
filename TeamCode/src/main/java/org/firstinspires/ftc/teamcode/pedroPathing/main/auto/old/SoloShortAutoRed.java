package org.firstinspires.ftc.teamcode.pedroPathing.main.auto.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "red solo short auto")
public class SoloShortAutoRed extends OpMode {
    SoloShortAuto opMode = new SoloShortAuto();
    @Override
    public void init() {
        opMode = new SoloShortAuto();
        opMode.init(hardwareMap, telemetry, false);
    }

    @Override
    public void loop() {
        opMode.loop();
    }
}
