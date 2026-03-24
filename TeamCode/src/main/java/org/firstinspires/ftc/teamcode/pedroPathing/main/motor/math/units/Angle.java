package org.firstinspires.ftc.teamcode.pedroPathing.main.motor.math.units;

import org.firstinspires.ftc.teamcode.pedroPathing.main.motor.GoBILDAMotorTypes;

public class Angle {
    private double valueInRadians;
    private GoBILDAMotorTypes motorType;
    public static double radiansToTicks(double radians, GoBILDAMotorTypes motorType) {
        return radians * motorType.getTicksPerOutputRev() / (2 * Math.PI);
    }
    public static double radiansToTicks(double radians, double ticksPerOutputRev) {
        return (radians * ticksPerOutputRev) / (2 * Math.PI);
    }
    public static double ticksToRadians(GoBILDAMotorTypes motorType, int ticks) {
        return ticks * (2 * Math.PI) / motorType.getTicksPerOutputRev();
    }
    public static double ticksToRadians(double ticks, double ticksPerOutputRev) {
        return (ticks * (2 * Math.PI)) / ticksPerOutputRev;
    }
    public Angle(double value, AngleUnit unit, GoBILDAMotorTypes motorType) {
        this.motorType = motorType;
        if (unit == AngleUnit.RADIANS) {
            this.valueInRadians = value;
        } else if (unit == AngleUnit.TICKS) {
            this.valueInRadians = ticksToRadians(motorType, (int) value);
        }
    }
        public double get(AngleUnit unit) {
            if (unit == AngleUnit.RADIANS) {
                return valueInRadians;
            } else if (unit == AngleUnit.TICKS) {
                return radiansToTicks(valueInRadians, motorType);
            }
            return 0;
        }

}
