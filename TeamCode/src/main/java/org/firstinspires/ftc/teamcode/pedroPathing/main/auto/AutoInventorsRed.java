package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "inventors red")
public class AutoInventorsRed extends OpMode {
    AutoInventors auto = new AutoInventors();
    @Override
    public void init() {
        auto.init(hardwareMap, telemetry, false);
    }

    @Override
    public void loop() {
        auto.loop();
    }
}
