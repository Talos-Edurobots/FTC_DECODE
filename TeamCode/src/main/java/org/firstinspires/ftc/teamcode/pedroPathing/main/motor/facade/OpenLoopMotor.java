package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.facade;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.MetaMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.config.MotorLimits;

public class OpenLoopMotor {
    private final MetaMotor hardware;

    public OpenLoopMotor(String hardwareName,
                         DcMotorSimple.Direction direction,
                         DcMotor.ZeroPowerBehavior zeroPowerBehavior,
                         MotorLimits limits) {
        this(new MetaMotor(), hardwareName, direction, zeroPowerBehavior, limits);
    }

    public OpenLoopMotor(MetaMotor hardware,
                         String hardwareName,
                         DcMotorSimple.Direction direction,
                         DcMotor.ZeroPowerBehavior zeroPowerBehavior,
                         MotorLimits limits) {
        this.hardware = hardware;
        this.hardware.hwName(hardwareName);
        this.hardware.direction(direction);
        this.hardware.zeroPowerBehavior(zeroPowerBehavior);
        this.hardware.maxPower(limits.getMaxPower());
        this.hardware.currentAlert(limits.getCurrentAlertAmps());
    }

    public OpenLoopMotor(MetaMotor hardware) {
        this.hardware = hardware;
    }

    public void init(HardwareMap hardwareMap) {
        hardware.init(hardwareMap);
        hardware.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        hardware.setPower(power);
    }

    public double getPower() {
        return hardware.getPower();
    }

    public double getCurrentAmps() {
        return hardware.getCurrentAmps();
    }

    public MetaMotor getHardware() {
        return hardware;
    }
}
