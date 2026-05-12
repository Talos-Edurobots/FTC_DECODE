package org.firstinspires.ftc.teamcode.pedroPathing.main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MainBlue TeleOp", group = "!main")
public class MainBlue extends OpMode {
    private final MainTeleOp teleOp = new MainTeleOp();

    @Override public void init() { teleOp.init(this, true); }
    @Override public void init_loop() { teleOp.init_loop(); }
    @Override public void start() { teleOp.start(); }
    @Override public void loop() { teleOp.loop(); }
    @Override public void stop() { teleOp.stop(blackboard); }
}
