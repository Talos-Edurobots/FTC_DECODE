package org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry;

import java.util.Arrays;

public final class LoopTimeStats {
    private static final int BIN_WIDTH_MICROS = 250;
    private static final int MAX_TRACKED_MICROS = 250_000;
    private static final int OVERFLOW_BIN = MAX_TRACKED_MICROS / BIN_WIDTH_MICROS + 1;
    private static final int BIN_COUNT = OVERFLOW_BIN + 1;

    private final int[] histogram = new int[BIN_COUNT];
    private long sampleCount;
    private double totalSeconds;
    private double worstSeconds;

    public void reset() {
        Arrays.fill(histogram, 0);
        sampleCount = 0;
        totalSeconds = 0.0;
        worstSeconds = 0.0;
    }

    public void record(double loopSeconds) {
        if (loopSeconds <= 0.0 || Double.isNaN(loopSeconds) || Double.isInfinite(loopSeconds)) {
            return;
        }

        sampleCount++;
        totalSeconds += loopSeconds;
        worstSeconds = Math.max(worstSeconds, loopSeconds);
        histogram[getBin(loopSeconds)]++;
    }

    public long getSampleCount() {
        return sampleCount;
    }

    public Snapshot snapshot() {
        if (sampleCount == 0) {
            return Snapshot.empty();
        }

        long onePercentSamples = tailSampleCount(0.01);
        long pointOnePercentSamples = tailSampleCount(0.001);
        TailScan scan = scanSlowestTails(onePercentSamples, pointOnePercentSamples);

        return new Snapshot(
                sampleCount,
                totalSeconds * 1000.0 / sampleCount,
                worstSeconds * 1000.0,
                scan.onePercentMillis,
                scan.pointOnePercentMillis,
                scan.onePercentAverageMillis,
                scan.pointOnePercentAverageMillis
        );
    }

    private int getBin(double loopSeconds) {
        long micros = (long) (loopSeconds * 1_000_000.0 + 0.5);
        long bin = (micros + BIN_WIDTH_MICROS - 1) / BIN_WIDTH_MICROS;
        if (bin >= OVERFLOW_BIN) {
            return OVERFLOW_BIN;
        }
        return (int) Math.max(0, bin);
    }

    private long tailSampleCount(double fraction) {
        return Math.max(1, (long) Math.ceil(sampleCount * fraction));
    }

    private TailScan scanSlowestTails(long onePercentSamples, long pointOnePercentSamples) {
        long seen = 0;
        double onePercentSumMillis = 0.0;
        double pointOnePercentSumMillis = 0.0;
        double onePercentMillis = 0.0;
        double pointOnePercentMillis = 0.0;
        boolean foundOnePercent = false;
        boolean foundPointOnePercent = false;

        for (int bin = BIN_COUNT - 1; bin >= 0; bin--) {
            int binSamples = histogram[bin];
            if (binSamples == 0) {
                continue;
            }

            double binMillis = getBinUpperMillis(bin);
            long before = seen;
            seen += binSamples;

            long onePercentNeeded = Math.max(0, onePercentSamples - before);
            if (onePercentNeeded > 0) {
                onePercentSumMillis += binMillis * Math.min(onePercentNeeded, binSamples);
            }
            if (!foundOnePercent && seen >= onePercentSamples) {
                onePercentMillis = binMillis;
                foundOnePercent = true;
            }

            long pointOnePercentNeeded = Math.max(0, pointOnePercentSamples - before);
            if (pointOnePercentNeeded > 0) {
                pointOnePercentSumMillis += binMillis * Math.min(pointOnePercentNeeded, binSamples);
            }
            if (!foundPointOnePercent && seen >= pointOnePercentSamples) {
                pointOnePercentMillis = binMillis;
                foundPointOnePercent = true;
            }

            if (foundOnePercent && foundPointOnePercent) {
                break;
            }
        }

        return new TailScan(
                onePercentMillis,
                pointOnePercentMillis,
                onePercentSumMillis / onePercentSamples,
                pointOnePercentSumMillis / pointOnePercentSamples
        );
    }

    private double getBinUpperMillis(int bin) {
        if (bin == OVERFLOW_BIN) {
            return worstSeconds * 1000.0;
        }
        return bin * BIN_WIDTH_MICROS / 1000.0;
    }

    private static final class TailScan {
        private final double onePercentMillis;
        private final double pointOnePercentMillis;
        private final double onePercentAverageMillis;
        private final double pointOnePercentAverageMillis;

        private TailScan(double onePercentMillis,
                         double pointOnePercentMillis,
                         double onePercentAverageMillis,
                         double pointOnePercentAverageMillis) {
            this.onePercentMillis = onePercentMillis;
            this.pointOnePercentMillis = pointOnePercentMillis;
            this.onePercentAverageMillis = onePercentAverageMillis;
            this.pointOnePercentAverageMillis = pointOnePercentAverageMillis;
        }
    }

    public static final class Snapshot {
        public final long sampleCount;
        public final double averageMillis;
        public final double worstMillis;
        public final double p99Millis;
        public final double p999Millis;
        public final double onePercentLowAverageMillis;
        public final double pointOnePercentLowAverageMillis;

        private Snapshot(long sampleCount,
                         double averageMillis,
                         double worstMillis,
                         double p99Millis,
                         double p999Millis,
                         double onePercentLowAverageMillis,
                         double pointOnePercentLowAverageMillis) {
            this.sampleCount = sampleCount;
            this.averageMillis = averageMillis;
            this.worstMillis = worstMillis;
            this.p99Millis = p99Millis;
            this.p999Millis = p999Millis;
            this.onePercentLowAverageMillis = onePercentLowAverageMillis;
            this.pointOnePercentLowAverageMillis = pointOnePercentLowAverageMillis;
        }

        private static Snapshot empty() {
            return new Snapshot(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        public double onePercentLowHertz() {
            return hertzFromMillis(onePercentLowAverageMillis);
        }

        public double pointOnePercentLowHertz() {
            return hertzFromMillis(pointOnePercentLowAverageMillis);
        }

        private static double hertzFromMillis(double millis) {
            return millis > 0.0 ? 1000.0 / millis : 0.0;
        }
    }
}
