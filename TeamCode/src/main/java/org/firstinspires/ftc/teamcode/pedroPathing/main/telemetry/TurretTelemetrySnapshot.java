package org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry;

public final class TurretTelemetrySnapshot {
    public final String controlMode;
    public final double angleToGoalRadians;
    public final double targetAngleRadians;
    public final double measuredAngleRadians;
    public final double measuredVelocityTicksPerSecond;
    public final double appliedPower;
    public final double referenceVelocityTicksPerSecond;
    public final double referenceAccelerationTicksPerSecondSquared;
    public final boolean overCurrent;
    public final boolean atLowerLimit;
    public final boolean atUpperLimit;
    public final Double currentAmps;

    public TurretTelemetrySnapshot(String controlMode,
                                   double angleToGoalRadians,
                                   double targetAngleRadians,
                                   double measuredAngleRadians,
                                   double measuredVelocityTicksPerSecond,
                                   double appliedPower,
                                   double referenceVelocityTicksPerSecond,
                                   double referenceAccelerationTicksPerSecondSquared,
                                   boolean overCurrent,
                                   boolean atLowerLimit,
                                   boolean atUpperLimit,
                                   Double currentAmps) {
        this.controlMode = controlMode;
        this.angleToGoalRadians = angleToGoalRadians;
        this.targetAngleRadians = targetAngleRadians;
        this.measuredAngleRadians = measuredAngleRadians;
        this.measuredVelocityTicksPerSecond = measuredVelocityTicksPerSecond;
        this.appliedPower = appliedPower;
        this.referenceVelocityTicksPerSecond = referenceVelocityTicksPerSecond;
        this.referenceAccelerationTicksPerSecondSquared = referenceAccelerationTicksPerSecondSquared;
        this.overCurrent = overCurrent;
        this.atLowerLimit = atLowerLimit;
        this.atUpperLimit = atUpperLimit;
        this.currentAmps = currentAmps;
    }
}
