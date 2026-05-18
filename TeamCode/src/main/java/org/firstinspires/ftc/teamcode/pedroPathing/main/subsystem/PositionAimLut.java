package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.pedropathing.geometry.Pose;

import java.util.Arrays;
import java.util.Comparator;

/**
 * Position-based turret aiming LUT that maps robot field position to a virtual target point.
 */
public final class PositionAimLut {
    private static final double EXACT_MATCH_DISTANCE_EPSILON = 1e-6;

    private final Sample[] samples;

    public PositionAimLut(Sample... samples) {
        this.samples = samples != null ? samples.clone() : new Sample[0];
    }

    public boolean hasSamples() {
        return samples.length > 0;
    }

    public Pose getVirtualAimPoint(Pose robotPose, int neighborCount) {
        if (robotPose == null || !hasSamples()) {
            return null;
        }

        WeightedSample[] weightedSamples = new WeightedSample[samples.length];
        for (int i = 0; i < samples.length; i++) {
            Sample sample = samples[i];
            double dx = robotPose.getX() - sample.robotX;
            double dy = robotPose.getY() - sample.robotY;
            double distance = Math.hypot(dx, dy);
            if (distance < EXACT_MATCH_DISTANCE_EPSILON) {
                return new Pose(sample.aimX, sample.aimY);
            }
            weightedSamples[i] = new WeightedSample(distance, sample);
        }

        Arrays.sort(weightedSamples, Comparator.comparingDouble(weighted -> weighted.distance));
        int neighborLimit = Math.min(Math.max(1, neighborCount), weightedSamples.length);

        double totalWeight = 0.0;
        double weightedAimX = 0.0;
        double weightedAimY = 0.0;
        for (int i = 0; i < neighborLimit; i++) {
            WeightedSample weightedSample = weightedSamples[i];
            double weight = 1.0 / (weightedSample.distance * weightedSample.distance);
            totalWeight += weight;
            weightedAimX += weightedSample.sample.aimX * weight;
            weightedAimY += weightedSample.sample.aimY * weight;
        }

        if (totalWeight <= 0.0) {
            return null;
        }

        return new Pose(weightedAimX / totalWeight, weightedAimY / totalWeight);
    }

    public static Sample sample(double robotX, double robotY, double aimX, double aimY) {
        return new Sample(robotX, robotY, aimX, aimY);
    }

    public static final class Sample {
        private final double robotX;
        private final double robotY;
        private final double aimX;
        private final double aimY;

        private Sample(double robotX, double robotY, double aimX, double aimY) {
            this.robotX = robotX;
            this.robotY = robotY;
            this.aimX = aimX;
            this.aimY = aimY;
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
