package org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry;

public final class TurretTelemetrySnapshot {
    public final String controlMode;
    public final boolean lutEnabled;
    public final boolean usingVirtualAimPoint;
    public final double angleToGoalRadians;
    public final double targetAngleRadians;
    public final Double aimPointX;
    public final Double aimPointY;
    public final int positionTicks;
    public final double measuredAngleRadians;
    public final double measuredVelocityTicksPerSecond;
    public final double appliedPower;
    public final double referencePositionTicks;
    public final double referenceVelocityTicksPerSecond;
    public final double referenceAccelerationTicksPerSecondSquared;
    public final boolean overCurrent;
    public final boolean atLowerLimit;
    public final boolean atUpperLimit;
    public final Double currentAmps;

    public TurretTelemetrySnapshot(String controlMode,
                                   boolean lutEnabled,
                                   boolean usingVirtualAimPoint,
                                   double angleToGoalRadians,
                                   double targetAngleRadians,
                                   Double aimPointX,
                                   Double aimPointY,
                                   int positionTicks,
                                   double measuredAngleRadians,
                                   double measuredVelocityTicksPerSecond,
                                   double appliedPower,
                                   double referencePositionTicks,
                                   double referenceVelocityTicksPerSecond,
                                   double referenceAccelerationTicksPerSecondSquared,
                                   boolean overCurrent,
                                   boolean atLowerLimit,
                                   boolean atUpperLimit,
                                   Double currentAmps) {
        this.controlMode = controlMode;
        this.lutEnabled = lutEnabled;
        this.usingVirtualAimPoint = usingVirtualAimPoint;
        this.angleToGoalRadians = angleToGoalRadians;
        this.targetAngleRadians = targetAngleRadians;
        this.aimPointX = aimPointX;
        this.aimPointY = aimPointY;
        this.positionTicks = positionTicks;
        this.measuredAngleRadians = measuredAngleRadians;
        this.measuredVelocityTicksPerSecond = measuredVelocityTicksPerSecond;
        this.appliedPower = appliedPower;
        this.referencePositionTicks = referencePositionTicks;
        this.referenceVelocityTicksPerSecond = referenceVelocityTicksPerSecond;
        this.referenceAccelerationTicksPerSecondSquared = referenceAccelerationTicksPerSecondSquared;
        this.overCurrent = overCurrent;
        this.atLowerLimit = atLowerLimit;
        this.atUpperLimit = atUpperLimit;
        this.currentAmps = currentAmps;
    }
}
