package org.team100.lib.prrts;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.unc.robotics.prrts.PRRTStar;
import edu.unc.robotics.prrts.Path;
import edu.unc.robotics.prrts.example.arena.HolonomicArena;

@SuppressWarnings("unused")
public class TestBestPath {

    /** Cribbed from ArenaFrame */
    @Test
    public void testTime() {
        final HolonomicArena arena = new HolonomicArena();
        double[] init = { 7.0, 1.0};

        final PRRTStar rrtStar = new PRRTStar(arena, arena, init, 6.0, 4);

        Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
        Thread.currentThread().getThreadGroup().setMaxPriority(Thread.MIN_PRIORITY);

        rrtStar.runForDurationMS(20);

        int steps = rrtStar.getStepNo();
        // assertEquals(2300, steps, 400); // very approximately equal
        assertEquals(3600, steps, 1000); // AtomicReference speeds it up a lot but there's a lot of variance

        int nodes = 0;
        for (var n : rrtStar.getNodes()) {
            nodes++;
        }
        // assertEquals(2300, nodes, 300);
        assertEquals(3600, nodes, 800);// AtomicReference speeds it up a lot
        Path bestPath = rrtStar.getBestPath();
        assertEquals(6.1, bestPath.get_dist(), 0.5); // very approximate
    }

    @Test
    public void testSteps() {
        final HolonomicArena arena = new HolonomicArena();
        double[] init = { 7.0, 1.0 };

        final PRRTStar rrtStar = new PRRTStar(arena, arena, init, 6.0, 4);

        Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
        Thread.currentThread().getThreadGroup().setMaxPriority(Thread.MIN_PRIORITY);

        rrtStar.runSamples(500);

        int steps = rrtStar.getStepNo();
        assertEquals(504, steps, 10);
        int nodes = 0;
        for (var n : rrtStar.getNodes()) {
            nodes++;
        }
        assertEquals(500, nodes, 10);
        Path bestPath = rrtStar.getBestPath();
        // weird how much variation there is in this result
        assertEquals(8, bestPath.get_dist(), 2);
    }

}
