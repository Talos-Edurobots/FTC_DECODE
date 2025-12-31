package org.firstinspires.ftc.teamcode.pedroPathing.main.config;

public enum GoBildaMotor {

    // Yellow Jacket Planetary Motors (312 RPM series and below)

    MOTOR_117_RPM  (117,  ((((1+((double) 46 /17))) * (1+((double) 46 /17))) * (1+((double) 46 /17)))),
    MOTOR_312_RPM  (312,  ((((1+((double) 46 /17))) * (1+((double) 46 /11))) * 28)),
    MOTOR_1150_RPM (1150, (1+((double) 46 /11))),
    MOTOR_6000_RPM (6000, 1);

    /** Encoder counts per motor shaft revolution */
    public static final double ENCODER_TICKS_PER_MOTOR_REV = 28;

    private final double rpm;
    private final double gearRatio;

    GoBildaMotor(double rpm, double gearRatio) {
        this.rpm = rpm;
        this.gearRatio = gearRatio;
    }

    /** Nominal output RPM at 12V */
    public double getRpm() {
        return rpm;
    }

    /** Gear reduction (motor revolutions per output shaft revolution) */
    public double getGearRatio() {
        return gearRatio;
    }

    /** Encoder ticks per output shaft revolution */
    public double getTicksPerOutputRev() {
        return ENCODER_TICKS_PER_MOTOR_REV * gearRatio;
    }

    /** Encoder ticks per degree of output shaft rotation */
    public double getTicksPerDegree() {
        return getTicksPerOutputRev() / 360.0;
    }

    /** Encoder ticks per radian of output shaft rotation */
    public double getTicksPerRadian() {
        return getTicksPerOutputRev() / (2.0 * Math.PI);
    }
}
