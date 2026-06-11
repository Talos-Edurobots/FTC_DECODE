package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.main.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MetaMotor;

public class  Hang {
    private Servo left, right;
    HardwareMap hwmap;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private double targetPositionTicks = 0.0;

    public void init(HardwareMap hwmap) {
        this.hwmap = hwmap;
        left = hwmap.servo.get("leftHang");
        right = hwmap.servo.get("rightHang");
    }

    public void setState(boolean isActivate) {
        left.setPosition(isActivate?1:0);
        right.setPosition(isActivate?1:0);
    }
}
