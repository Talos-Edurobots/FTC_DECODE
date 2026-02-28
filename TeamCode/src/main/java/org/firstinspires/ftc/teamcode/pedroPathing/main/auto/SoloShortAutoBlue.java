package org.firstinspires.ftc.teamcode.pedroPathing.main.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.PPConstants;

@Autonomous(name = "blue solo short auto")
public class SoloShortAutoBlue extends OpMode {
    SoloShortAuto opMode = new SoloShortAuto();
    @Override
    public void init() {
        opMode = opMode.get();
        opMode.init(hardwareMap, telemetry, true);
    }

    @Override
    public void loop() {
        opMode.loop();
    }
}
