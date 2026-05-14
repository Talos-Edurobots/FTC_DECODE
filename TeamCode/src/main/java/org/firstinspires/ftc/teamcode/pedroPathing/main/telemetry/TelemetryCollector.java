package org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

public final class TelemetryCollector {
    private final TelemetryMode activeMode;
    private final double nowSeconds;
    private final boolean modeChanged;
    private final Map<String, FieldState> fieldStates;
    private final LinkedHashMap<String, Object> pendingFields = new LinkedHashMap<>();

    TelemetryCollector(TelemetryMode activeMode,
                       double nowSeconds,
                       boolean modeChanged,
                       Map<String, FieldState> fieldStates) {
        this.activeMode = activeMode;
        this.nowSeconds = nowSeconds;
        this.modeChanged = modeChanged;
        this.fieldStates = fieldStates;
    }

    public TelemetryMode getActiveMode() {
        return activeMode;
    }

    public double getNowSeconds() {
        return nowSeconds;
    }

    public void add(String section,
                    String key,
                    Object value,
                    TelemetryMode minimumMode,
                    TelemetryCostClass costClass) {
        add(section, key, value, minimumMode, costClass, TelemetryPublishPolicy.ALWAYS);
    }

    public void add(String section,
                    String key,
                    Object value,
                    TelemetryMode minimumMode,
                    TelemetryCostClass costClass,
                    TelemetryPublishPolicy policy) {
        if (value == null || !activeMode.includes(minimumMode)) {
            return;
        }

        String fieldKey = section + "." + key;
        FieldState state = fieldStates.computeIfAbsent(fieldKey, ignored -> new FieldState());
        if (!shouldPublish(state, value, policy)) {
            return;
        }

        state.lastValue = value;
        state.hasPublished = true;
        state.lastMode = activeMode;
        state.costClass = costClass;
        pendingFields.put(fieldKey, value);
    }

    public void publish(TelemetryManager panelsTelemetry, Telemetry telemetry) {
        for (Map.Entry<String, Object> entry : pendingFields.entrySet()) {
            panelsTelemetry.addData(entry.getKey(), entry.getValue());
            telemetry.addData(entry.getKey(), entry.getValue());
        }
        panelsTelemetry.update(telemetry);
    }

    private boolean shouldPublish(FieldState state, Object value, TelemetryPublishPolicy policy) {
        switch (policy) {
            case ON_CHANGE:
                return !state.hasPublished || !value.equals(state.lastValue);
            case ON_MODE_ENTRY:
                return !state.hasPublished || modeChanged || state.lastMode != activeMode;
            case ALWAYS:
            default:
                return true;
        }
    }

    static final class FieldState {
        private Object lastValue;
        private boolean hasPublished;
        private TelemetryMode lastMode;
        private TelemetryCostClass costClass;
    }
}
