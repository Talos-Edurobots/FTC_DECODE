package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.GoBILDAMotorTypes;

public class Velocity {
    private double valueInTicksPerSecond;
    private GoBILDAMotorTypes motorType;
    public static double ticksPerSecondToRpm(int ticks, GoBILDAMotorTypes motorType) {
        return (ticks / motorType.getTicksPerOutputRev()) * 60.0;
    }
    public static double ticksPerSecondToRpm(int ticks, double ticksPerOutputRev) {
        return (ticks / ticksPerOutputRev)*60.0;
    }
    public static double rpmToTicksPerSecond(double rpm, GoBILDAMotorTypes motorType) {
        return (rpm / 60.0) * motorType.getTicksPerOutputRev();
    }
    public static double rpmToTicksPerSecond(double rpm, double ticksPerOutputRev) {
        return (rpm / 60.0) * ticksPerOutputRev;
    }
    public Velocity(double value, VelocityUnit unit, GoBILDAMotorTypes motorType) {
        this.motorType = motorType;
        if (unit == VelocityUnit.TICKS_PER_SECOND) {
            this.valueInTicksPerSecond = value;
        } else if (unit == VelocityUnit.RPM) {
            this.valueInTicksPerSecond = rpmToTicksPerSecond(value, motorType);
        }
    }
    public double get(VelocityUnit unit) {
        if (unit == VelocityUnit.TICKS_PER_SECOND) {
            return valueInTicksPerSecond;
        } else if (unit == VelocityUnit.RPM) {
            return ticksPerSecondToRpm((int) valueInTicksPerSecond, motorType);
        }
        return 0;
    }

}
