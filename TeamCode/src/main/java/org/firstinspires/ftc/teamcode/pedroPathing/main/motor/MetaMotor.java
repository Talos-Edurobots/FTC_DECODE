package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MetaMotor {
    private DcMotorEx motor;
    private double currentAlert = Double.POSITIVE_INFINITY;
    private String hwName;
    private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior =
            DcMotor.ZeroPowerBehavior.FLOAT;
    private double maxPower = 1.0;

    public void hwName(String hwName) {
        this.hwName = hwName;
    }
    public void direction(DcMotorSimple.Direction direction) {
        this.direction = direction;
    }
    public void zeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    public void currentAlert(double currentAlert) {
        this.currentAlert = currentAlert;
    }

    public void maxPower(double maxPower) {
        this.maxPower = Range.clip(maxPower, 0.0, 1.0);
    }

    public MetaMotor() {}

    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, hwName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        if (Double.isFinite(currentAlert)) {
            motor.setCurrentAlert(currentAlert, CurrentUnit.AMPS);
        }
    }

    public DcMotorEx getMotor() {
        requireInitialized();
        return motor;
    }

    public void setPower(double power) {
        requireInitialized();
        motor.setPower(Range.clip(power, -maxPower, maxPower));
    }

    public double getPower() {
        requireInitialized();
        return motor.getPower();
    }

    public void setMode(RunMode runMode) {
        requireInitialized();
        motor.setMode(runMode);
    }

    public RunMode getMode() {
        requireInitialized();
        return motor.getMode();
    }

    public int getCurrentPositionTicks() {
        requireInitialized();
        return motor.getCurrentPosition();
    }

    public double getVelocityTicksPerSecond() {
        requireInitialized();
        return motor.getVelocity();
    }

    public double getCurrentAmps() {
        requireInitialized();
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isOverCurrent() {
        requireInitialized();
        return motor.isOverCurrent();
    }

    public void setCurrentAlert(double currentAlertAmps) {
        this.currentAlert = currentAlertAmps;
        if (motor != null && Double.isFinite(currentAlertAmps)) {
            motor.setCurrentAlert(currentAlertAmps, CurrentUnit.AMPS);
        }
    }

    public double getCurrentAlert() {
        return currentAlert;
    }

    public String getHardwareName() {
        return hwName;
    }

    public DcMotorSimple.Direction getDirection() {
        return direction;
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    public double getMaxPower() {
        return maxPower;
    }

    private void requireInitialized() {
        if (motor == null) {
            throw new IllegalStateException("MetaMotor has not been initialized");
        }
    }
}
