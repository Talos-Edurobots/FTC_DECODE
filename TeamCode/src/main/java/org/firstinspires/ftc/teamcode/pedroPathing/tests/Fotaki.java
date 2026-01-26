package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.lights.LightObject;
import com.bylazar.lights.LightsManager;
import com.bylazar.lights.PanelsLights;
import com.bylazar.panels.Panels;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Configurable
@TeleOp
public class Fotaki extends LinearOpMode {
    static double color = 0;
    LightsManager light = PanelsLights.INSTANCE.getLights();
    @Override
    public void runOpMode() throws InterruptedException {
        Servo led;
        led = hardwareMap.servo.get("led");
        light.setLastLights((List<LightObject>) led);
        waitForStart();
        while (opModeIsActive()) {
            led.setPosition(color);
            sleep(100);
        }

    }
}
