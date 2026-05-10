package org.firstinspires.ftc.teamcode.pedroPathing.main.motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

/**
 * Reusable multi-motor mechanism wrapper.
 *
 * <p>The primary motor remains the single feedback/control source. Once the
 * primary motor computes a power output, that output can be fanned out to every
 * motor in the group.
 */
public final class MotorGroup {
    private final MotorConfig[] motors;
    private final int primaryIndex;

    public MotorGroup(MotorConfig... motors) {
        this(0, motors);
    }

    public MotorGroup(int primaryIndex, MotorConfig... motors) {
        if (motors == null || motors.length == 0) {
            throw new IllegalArgumentException("MotorGroup requires at least one motor");
        }
        if (primaryIndex < 0 || primaryIndex >= motors.length) {
            throw new IllegalArgumentException("Primary motor index is out of bounds");
        }
        this.motors = Arrays.copyOf(motors, motors.length);
        this.primaryIndex = primaryIndex;
    }

    public void init(HardwareMap hardwareMap) {
        for (MotorConfig motor : motors) {
            motor.init(hardwareMap);
        }
    }

    public MotorConfig getPrimaryMotor() {
        return motors[primaryIndex];
    }

    public MotorConfig getMotor(int index) {
        return motors[index];
    }

    public MotorConfig[] getMotors() {
        return Arrays.copyOf(motors, motors.length);
    }

    public int size() {
        return motors.length;
    }

    public void setPower(double power) {
        for (MotorConfig motor : motors) {
            motor.setPower(power);
        }
    }

    public void stop() {
        setPower(0.0);
    }

    public void applyPrimaryPowerToAll() {
        setPower(getPrimaryMotor().getPower());
    }

    public void setVelocityTicksPerSecond(double ticksPerSecond) {
        getPrimaryMotor().setVelocityTicksPerSecond(ticksPerSecond);
    }

    public void updateVelocityPIDF() {
        getPrimaryMotor().updateVelocityPIDF();
        applyPrimaryPowerToAll();
    }
}
