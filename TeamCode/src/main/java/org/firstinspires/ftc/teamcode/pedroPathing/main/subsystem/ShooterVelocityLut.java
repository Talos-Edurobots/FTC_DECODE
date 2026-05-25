package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * One-dimensional interpolated LUT for distance-to-shooter-velocity calibration.
 */
public final class ShooterVelocityLut {
    private static final double EXACT_MATCH_DISTANCE_EPSILON = 1e-6;

    private final List<Sample> samples = new ArrayList<>();

    public ShooterVelocityLut(Sample... initialSamples) {
        if (initialSamples != null) {
            for (Sample sample : initialSamples) {
                if (sample != null) {
                    putSample(sample.distanceFromGoal, sample.targetVelocityTicksPerSecond);
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

    public void putSample(double distanceFromGoal, double targetVelocityTicksPerSecond) {
        validateFinite(distanceFromGoal, "distanceFromGoal");
        validateFinite(targetVelocityTicksPerSecond, "targetVelocityTicksPerSecond");

        for (int i = 0; i < samples.size(); i++) {
            Sample sample = samples.get(i);
            if (Math.abs(sample.distanceFromGoal - distanceFromGoal) < EXACT_MATCH_DISTANCE_EPSILON) {
                samples.set(i, sample(distanceFromGoal, targetVelocityTicksPerSecond));
                sortSamples();
                return;
            }
        }

        samples.add(sample(distanceFromGoal, targetVelocityTicksPerSecond));
        sortSamples();
    }

    public double getTargetVelocity(double distanceFromGoal) {
        validateFinite(distanceFromGoal, "distanceFromGoal");
        if (samples.isEmpty()) {
            return Double.NaN;
        }
        if (samples.size() == 1) {
            return samples.get(0).targetVelocityTicksPerSecond;
        }

        Sample first = samples.get(0);
        if (distanceFromGoal <= first.distanceFromGoal) {
            return first.targetVelocityTicksPerSecond;
        }

        Sample last = samples.get(samples.size() - 1);
        if (distanceFromGoal >= last.distanceFromGoal) {
            return last.targetVelocityTicksPerSecond;
        }

        for (int i = 0; i < samples.size() - 1; i++) {
            Sample low = samples.get(i);
            Sample high = samples.get(i + 1);
            if (Math.abs(distanceFromGoal - low.distanceFromGoal) < EXACT_MATCH_DISTANCE_EPSILON) {
                return low.targetVelocityTicksPerSecond;
            }
            if (distanceFromGoal <= high.distanceFromGoal) {
                double span = high.distanceFromGoal - low.distanceFromGoal;
                double t = (distanceFromGoal - low.distanceFromGoal) / span;
                return lerp(low.targetVelocityTicksPerSecond, high.targetVelocityTicksPerSecond, t);
            }
        }

        return last.targetVelocityTicksPerSecond;
    }

    public static double distanceToGoal(Pose robotPose, double goalX, double goalY) {
        if (robotPose == null) {
            return Double.NaN;
        }
        return distanceToGoal(robotPose.getX(), robotPose.getY(), goalX, goalY);
    }

    public static double distanceToGoal(double robotX, double robotY, double goalX, double goalY) {
        return Math.hypot(robotX - goalX, robotY - goalY);
    }

    public static Sample sample(double distanceFromGoal, double targetVelocityTicksPerSecond) {
        return new Sample(distanceFromGoal, targetVelocityTicksPerSecond);
    }

    private void sortSamples() {
        Collections.sort(samples, Comparator.comparingDouble(sample -> sample.distanceFromGoal));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static void validateFinite(double value, String name) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            throw new IllegalArgumentException(name + " must be finite");
        }
    }

    public static final class Sample {
        public final double distanceFromGoal;
        public final double targetVelocityTicksPerSecond;

        private Sample(double distanceFromGoal, double targetVelocityTicksPerSecond) {
            this.distanceFromGoal = distanceFromGoal;
            this.targetVelocityTicksPerSecond = targetVelocityTicksPerSecond;
        }
    }
}
