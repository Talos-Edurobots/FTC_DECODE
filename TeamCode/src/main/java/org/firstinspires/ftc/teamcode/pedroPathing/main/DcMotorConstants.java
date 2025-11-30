package org.firstinspires.ftc.teamcode.pedroPathing.main;

public final class DcMotorConstants {
    public static double Motor117GearRatio = ((((1+((double) 46 /17))) * (1+((double) 46 /17))) * (1+((double) 46 /17)) * 28);
    public static double Motor117EncoderResolution = 28 * Motor117GearRatio; // ticks per revolution of output shaft
    public static double Motor312GearRatio =((((1+((double) 46 /17))) * (1+((double) 46 /11))) * 28);
    public static double Motor312EncoderResolution = 28 * Motor312GearRatio; // ticks per revolution of output shaft
    public static double Motor1150GearRatio = 	((1+((double) 46 /11)) * 28);
    public static double Motor1150EncoderResolution = 28 * Motor1150GearRatio; // ticks per revolution of output shaft
    public static double Motor6000GearRatio = 1;
    public static double Motor6000EncoderResolution = 28 * Motor6000GearRatio; // ticks per revolution of output shaft
}
