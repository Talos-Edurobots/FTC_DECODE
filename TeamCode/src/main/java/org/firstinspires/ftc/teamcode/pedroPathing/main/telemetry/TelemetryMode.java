package org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry;

public enum TelemetryMode {
    OFF,
    COMPETITION,
    DEBUG,
    TRACE;

    public boolean includes(TelemetryMode minimumMode) {
        return this.ordinal() >= minimumMode.ordinal();
    }

    public TelemetryMode next() {
        TelemetryMode[] values = values();
        return values[(ordinal() + 1) % values.length];
    }
}
