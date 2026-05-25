package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Two-dimensional interpolated LUT for distance/velocity-to-hood-position calibration.
 */
public final class HoodAngleLut {
    public static final int DEFAULT_NEIGHBOR_COUNT = 4;
    private static final double EXACT_MATCH_EPSILON = 1e-6;
    private static final double MIN_HOOD_POSITION = 0.0;
    private static final double MAX_HOOD_POSITION = 0.5;

    private final List<Sample> samples = new ArrayList<>();

    public HoodAngleLut(Sample... initialSamples) {
        if (initialSamples != null) {
            for (Sample sample : initialSamples) {
                if (sample != null) {
                    putSample(
                            sample.distanceFromGoal,
                            sample.shooterVelocityTicksPerSecond,
                            sample.hoodServoPosition
                    );
                }
            }
        }
    }

    public boolean hasSamples() {
        return !samples.isEmpty();
    }

    public int getSampleCount() {
        return samples.size();
    }

    public Sample getSample(int index) {
        return samples.get(index);
    }

    public List<Sample> copySamples() {
        return new ArrayList<>(samples);
    }

    public void clear() {
        samples.clear();
    }

    public void putSample(double distanceFromGoal,
                          double shooterVelocityTicksPerSecond,
                          double hoodServoPosition) {
        validateFinite(distanceFromGoal, "distanceFromGoal");
        validateFinite(shooterVelocityTicksPerSecond, "shooterVelocityTicksPerSecond");
        validateFinite(hoodServoPosition, "hoodServoPosition");

        for (int i = 0; i < samples.size(); i++) {
            Sample sample = samples.get(i);
            boolean sameDistance = Math.abs(sample.distanceFromGoal - distanceFromGoal)
                    < EXACT_MATCH_EPSILON;
            boolean sameVelocity = Math.abs(
                    sample.shooterVelocityTicksPerSecond - shooterVelocityTicksPerSecond
            ) < EXACT_MATCH_EPSILON;
            if (sameDistance && sameVelocity) {
                samples.set(i, sample(distanceFromGoal, shooterVelocityTicksPerSecond,
                        hoodServoPosition));
                return;
            }
        }

        samples.add(sample(distanceFromGoal, shooterVelocityTicksPerSecond, hoodServoPosition));
    }

    public double getHoodPosition(double distanceFromGoal, double shooterVelocityTicksPerSecond) {
        return getHoodPosition(distanceFromGoal, shooterVelocityTicksPerSecond,
                DEFAULT_NEIGHBOR_COUNT);
    }

    public double getHoodPosition(double distanceFromGoal,
                                  double shooterVelocityTicksPerSecond,
                                  int neighborCount) {
        validateFinite(distanceFromGoal, "distanceFromGoal");
        validateFinite(shooterVelocityTicksPerSecond, "shooterVelocityTicksPerSecond");
        if (samples.isEmpty()) {
            return Double.NaN;
        }
        if (samples.size() == 1) {
            return samples.get(0).hoodServoPosition;
        }

        double distanceScale = getDistanceScale();
        double velocityScale = getVelocityScale();
        WeightedSample[] weightedSamples = new WeightedSample[samples.size()];

        for (int i = 0; i < samples.size(); i++) {
            Sample sample = samples.get(i);
            double distanceDelta = distanceFromGoal - sample.distanceFromGoal;
            double velocityDelta = shooterVelocityTicksPerSecond
                    - sample.shooterVelocityTicksPerSecond;
            if (Math.abs(distanceDelta) < EXACT_MATCH_EPSILON
                    && Math.abs(velocityDelta) < EXACT_MATCH_EPSILON) {
                return sample.hoodServoPosition;
            }

            double normalizedDistanceDelta = distanceDelta / distanceScale;
            double normalizedVelocityDelta = velocityDelta / velocityScale;
            double distance = Math.hypot(normalizedDistanceDelta, normalizedVelocityDelta);
            weightedSamples[i] = new WeightedSample(distance, sample);
        }

        Arrays.sort(weightedSamples, (a, b) -> Double.compare(a.distance, b.distance));
        int neighborLimit = Math.min(Math.max(1, neighborCount), weightedSamples.length);

        double totalWeight = 0.0;
        double weightedPosition = 0.0;
        for (int i = 0; i < neighborLimit; i++) {
            WeightedSample weightedSample = weightedSamples[i];
            double weight = 1.0 / (weightedSample.distance * weightedSample.distance);
            totalWeight += weight;
            weightedPosition += weightedSample.sample.hoodServoPosition * weight;
        }

        if (totalWeight <= 0.0) {
            return Double.NaN;
        }
        return clipHoodPosition(weightedPosition / totalWeight);
    }

    public static Sample sample(double distanceFromGoal,
                                double shooterVelocityTicksPerSecond,
                                double hoodServoPosition) {
        return new Sample(
                distanceFromGoal,
                shooterVelocityTicksPerSecond,
                clipHoodPosition(hoodServoPosition)
        );
    }

    private double getDistanceScale() {
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        for (Sample sample : samples) {
            min = Math.min(min, sample.distanceFromGoal);
            max = Math.max(max, sample.distanceFromGoal);
        }
        return Math.max(max - min, 1.0);
    }

    private double getVelocityScale() {
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        for (Sample sample : samples) {
            min = Math.min(min, sample.shooterVelocityTicksPerSecond);
            max = Math.max(max, sample.shooterVelocityTicksPerSecond);
        }
        return Math.max(max - min, 1.0);
    }

    private static double clipHoodPosition(double hoodServoPosition) {
        return Math.max(MIN_HOOD_POSITION, Math.min(MAX_HOOD_POSITION, hoodServoPosition));
    }

    private static void validateFinite(double value, String name) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            throw new IllegalArgumentException(name + " must be finite");
        }
    }

    public static final class Sample {
        public final double distanceFromGoal;
        public final double shooterVelocityTicksPerSecond;
        public final double hoodServoPosition;

        private Sample(double distanceFromGoal,
                       double shooterVelocityTicksPerSecond,
                       double hoodServoPosition) {
            this.distanceFromGoal = distanceFromGoal;
            this.shooterVelocityTicksPerSecond = shooterVelocityTicksPerSecond;
            this.hoodServoPosition = hoodServoPosition;
        }
    }

    private static final class WeightedSample {
        private final double distance;
        private final Sample sample;

        private WeightedSample(double distance, Sample sample) {
            this.distance = distance;
            this.sample = sample;
        }
    }
}
