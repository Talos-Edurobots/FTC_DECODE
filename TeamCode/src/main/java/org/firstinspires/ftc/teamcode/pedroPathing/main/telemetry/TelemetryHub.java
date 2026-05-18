package org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class TelemetryHub {
    private static final double SLOW_PUBLISH_INTERVAL_SECONDS = 0.2;
    private static final double FAST_PUBLISH_INTERVAL_SECONDS = 0.0;

    private final List<TelemetryProvider> providers = new ArrayList<>();
    private final Map<String, TelemetryCollector.FieldState> fieldStates = new LinkedHashMap<>();
    private TelemetryMode mode = TelemetryMode.COMPETITION;
    private TelemetryMode lastPublishedMode = null;
    private double lastOutputTimeSeconds = Double.NEGATIVE_INFINITY;

    public void register(TelemetryProvider provider) {
        providers.add(provider);
    }

    public void clearProviders() {
        providers.clear();
    }

    public TelemetryMode getMode() {
        return mode;
    }

    public void setMode(TelemetryMode mode) {
        this.mode = mode;
    }

    public void cycleMode() {
        mode = mode.next();
    }

    public void publish(TelemetryManager panelsTelemetry, Telemetry telemetry, double nowSeconds) {
        if (!shouldPublish(nowSeconds)) {
            return;
        }

        boolean modeChanged = lastPublishedMode != mode;
        TelemetryCollector collector =
                new TelemetryCollector(mode, nowSeconds, modeChanged, fieldStates);

        for (TelemetryProvider provider : providers) {
            provider.collectTelemetry(collector, mode);
        }

        collector.publish(panelsTelemetry, telemetry);
        lastPublishedMode = mode;
        lastOutputTimeSeconds = nowSeconds;
    }

    private boolean shouldPublish(double nowSeconds) {
        if (lastPublishedMode != mode) {
            return true;
        }

        return nowSeconds - lastOutputTimeSeconds >= getPublishIntervalSeconds();
    }

    private double getPublishIntervalSeconds() {
        switch (mode) {
            case DEBUG:
            case TRACE:
                return FAST_PUBLISH_INTERVAL_SECONDS;
            case OFF:
            case COMPETITION:
            default:
                return SLOW_PUBLISH_INTERVAL_SECONDS;
        }
    }
}
