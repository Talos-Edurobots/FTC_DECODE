package org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class TelemetryHub {
    private final List<TelemetryProvider> providers = new ArrayList<>();
    private final Map<String, TelemetryCollector.FieldState> fieldStates = new LinkedHashMap<>();
    private TelemetryMode mode = TelemetryMode.COMPETITION;
    private TelemetryMode lastPublishedMode = null;

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
        boolean modeChanged = lastPublishedMode != mode;
        TelemetryCollector collector =
                new TelemetryCollector(mode, nowSeconds, modeChanged, fieldStates);

        for (TelemetryProvider provider : providers) {
            provider.collectTelemetry(collector, mode);
        }

        collector.publish(panelsTelemetry, telemetry);
        lastPublishedMode = mode;
    }
}
