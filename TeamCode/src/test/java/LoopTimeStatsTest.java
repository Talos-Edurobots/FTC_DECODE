import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.pedroPathing.main.telemetry.LoopTimeStats;
import org.junit.Test;

public class LoopTimeStatsTest {
    @Test
    public void snapshotReportsAverageWorstAndSlowTailAverages() {
        LoopTimeStats stats = new LoopTimeStats();
        for (int i = 0; i < 990; i++) {
            stats.record(0.010);
        }
        for (int i = 0; i < 9; i++) {
            stats.record(0.020);
        }
        stats.record(0.030);

        LoopTimeStats.Snapshot snapshot = stats.snapshot();

        assertEquals(1000, snapshot.sampleCount);
        assertEquals(10.2, snapshot.averageMillis, 1e-9);
        assertEquals(30.0, snapshot.worstMillis, 1e-9);
        assertEquals(20.0, snapshot.p99Millis, 1e-9);
        assertEquals(30.0, snapshot.p999Millis, 1e-9);
        assertEquals(21.0, snapshot.onePercentLowAverageMillis, 1e-9);
        assertEquals(30.0, snapshot.pointOnePercentLowAverageMillis, 1e-9);
        assertEquals(1000.0 / 21.0, snapshot.onePercentLowHertz(), 1e-9);
    }

    @Test
    public void resetClearsSamples() {
        LoopTimeStats stats = new LoopTimeStats();
        stats.record(0.010);
        stats.reset();

        LoopTimeStats.Snapshot snapshot = stats.snapshot();

        assertEquals(0, snapshot.sampleCount);
        assertEquals(0.0, snapshot.averageMillis, 1e-9);
        assertEquals(0.0, snapshot.worstMillis, 1e-9);
    }
}
