package org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry;

public final class ShooterTelemetrySnapshot {
    public final double targetVelocityTicksPerSecond;
    public final double measuredVelocityTicksPerSecond;
    public final double filteredVelocityTicksPerSecond;
    public final double appliedPower;
    public final double hoodAngle;
    public final boolean running;
    public final boolean busy;
    public final boolean impactDetected;
    public final boolean overCurrent;
    public final Double currentAmps;
    public final Double followerCurrentAmps;
    public final double batteryVoltage;

    public ShooterTelemetrySnapshot(double targetVelocityTicksPerSecond,
                                    double measuredVelocityTicksPerSecond,
                                    double filteredVelocityTicksPerSecond,
                                    double appliedPower,
                                    double hoodAngle,
                                    boolean running,
                                    boolean busy,
                                    boolean impactDetected,
                                    boolean overCurrent,
                                    Double currentAmps,
                                    Double followerCurrentAmps,
                                    double batteryVoltage) {
        this.targetVelocityTicksPerSecond = targetVelocityTicksPerSecond;
        this.measuredVelocityTicksPerSecond = measuredVelocityTicksPerSecond;
        this.filteredVelocityTicksPerSecond = filteredVelocityTicksPerSecond;
        this.appliedPower = appliedPower;
        this.hoodAngle = hoodAngle;
        this.running = running;
        this.busy = busy;
        this.impactDetected = impactDetected;
        this.overCurrent = overCurrent;
        this.currentAmps = currentAmps;
        this.followerCurrentAmps = followerCurrentAmps;
        this.batteryVoltage = batteryVoltage;
    }
}
