package org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry;

import java.util.function.Supplier;

public final class ThrottledValue<T> {
    private final double minIntervalSeconds;
    private double lastSampleTimeSeconds = Double.NEGATIVE_INFINITY;
    private T cachedValue;

    public ThrottledValue(double minIntervalSeconds) {
        this.minIntervalSeconds = Math.max(0.0, minIntervalSeconds);
    }

    public T get(double nowSeconds, Supplier<T> sampler) {
        if (cachedValue == null || nowSeconds - lastSampleTimeSeconds >= minIntervalSeconds) {
            cachedValue = sampler.get();
            lastSampleTimeSeconds = nowSeconds;
        }
        return cachedValue;
    }

    public void invalidate() {
        lastSampleTimeSeconds = Double.NEGATIVE_INFINITY;
        cachedValue = null;
    }
}
