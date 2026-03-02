package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;

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
