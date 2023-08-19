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

        final PRRTStar rrtStar = new PRRTStar(arena, arena, init);

        rrtStar.runForDurationMS(6.0, 20);

        int steps = rrtStar.getStepNo();
        assertEquals(3600, steps, 1000); 

        int nodes = 0;
        for (var n : rrtStar.getNodes()) {
            nodes++;
        }

        assertEquals(3600, nodes, 800);
        Path bestPath = rrtStar.getBestPath();
        assertEquals(6.1, bestPath.get_dist(), 0.5);
    }

    @Test
    public void testSteps() {
        final HolonomicArena arena = new HolonomicArena();
        double[] init = { 7.0, 1.0 };

        final PRRTStar rrtStar = new PRRTStar(arena, arena, init);

        rrtStar.runSamples(6.0, 500);

        int steps = rrtStar.getStepNo();
        assertEquals(504, steps, 10);
        int nodes = 0;
        for (var n : rrtStar.getNodes()) {
            nodes++;
        }
        assertEquals(500, nodes, 10);
        Path bestPath = rrtStar.getBestPath();
        assertEquals(6, bestPath.get_dist(), 2);
    }

}
