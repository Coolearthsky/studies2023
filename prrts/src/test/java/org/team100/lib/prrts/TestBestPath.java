package org.team100.lib.prrts;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.unc.robotics.prrts.ArrayState;
import edu.unc.robotics.prrts.PRRTStar;
import edu.unc.robotics.prrts.Path;
import edu.unc.robotics.prrts.example.arena.HolonomicArena;

/**
 * These tests are really dependent on what else the machine is doing, so they
 * should have wide bounds.
 */
@SuppressWarnings("unused")
public class TestBestPath {

    /** Cribbed from ArenaFrame */
    @Test
    public void testTime() {
        final HolonomicArena arena = new HolonomicArena();
        ArrayState init = new ArrayState(new double[] { 7.0, 1.0 });

        final PRRTStar<ArrayState> rrtStar = new PRRTStar<ArrayState>(arena, () -> arena, init);

        rrtStar.setGamma(6.0);
        rrtStar.setPerThreadRegionSampling(true);
        // rrtStar.setSamplesPerStep(100);

        Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
        Thread.currentThread().getThreadGroup().setMaxPriority(Thread.MIN_PRIORITY);

        rrtStar.runForDuration(4, 20);

        int steps = rrtStar.getStepNo();
        // assertEquals(2300, steps, 400); // very approximately equal
        assertEquals(3600, steps, 800); // AtomicReference speeds it up a lot but there's a lot of variance

        int nodes = 0;
        for (var n : rrtStar.getNodes()) {
            nodes++;
        }
        // assertEquals(2300, nodes, 300);
        assertEquals(3600, nodes, 800);// AtomicReference speeds it up a lot
        Path<ArrayState> bestPath = rrtStar.getBestPath();
        assertEquals(10.2, bestPath.dist, 0.5); // very approximate
    }

    @Test
    public void testSteps() {
        final HolonomicArena arena = new HolonomicArena();
        ArrayState init = new ArrayState(new double[] { 7.0, 1.0 });

        final PRRTStar<ArrayState> rrtStar = new PRRTStar<ArrayState>(arena, () -> arena, init);

        rrtStar.setGamma(6.0);
        rrtStar.setPerThreadRegionSampling(true);
        // rrtStar.setSamplesPerStep(100);

        Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
        Thread.currentThread().getThreadGroup().setMaxPriority(Thread.MIN_PRIORITY);

        rrtStar.runSamples(4, 500);

        int steps = rrtStar.getStepNo();
        assertEquals(504, steps, 10);
        int nodes = 0;
        for (var n : rrtStar.getNodes()) {
            nodes++;
        }
        assertEquals(500, nodes, 10);
        Path<ArrayState> bestPath = rrtStar.getBestPath();
        assertEquals(10.8, bestPath.dist, 0.5);
    }

}
