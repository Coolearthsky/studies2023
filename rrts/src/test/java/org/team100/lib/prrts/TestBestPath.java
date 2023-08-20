package org.team100.lib.prrts;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.unc.robotics.prrts.Runner;
import edu.unc.robotics.prrts.Sample;
import edu.unc.robotics.prrts.RRTStar;
import edu.unc.robotics.prrts.Path;
import edu.unc.robotics.prrts.example.arena.HolonomicArena;

@SuppressWarnings("unused")
public class TestBestPath {

    /** Cribbed from ArenaFrame */
    @Test
    public void testTime() {
        final HolonomicArena arena = new HolonomicArena();
        final RRTStar<HolonomicArena> worker = new RRTStar<>(arena, new Sample(arena), 6);
        final Runner rrtStar = new Runner(worker);

        rrtStar.runForDurationMS(20);

        int steps = rrtStar.getStepNo();
        assertEquals(2400, steps, 1000); 

        int nodes = 0;
        for (var n : rrtStar.getNodes()) {
            nodes++;
        }

        assertEquals(2400, nodes, 800);
        Path bestPath = rrtStar.getBestPath();
        assertEquals(16, bestPath.get_dist(), 1);
    }

    @Test
    public void testSteps() {
        final HolonomicArena arena = new HolonomicArena();
        final RRTStar<HolonomicArena> worker = new RRTStar<>(arena, new Sample(arena), 6);
        final Runner rrtStar = new Runner(worker);

        rrtStar.runSamples(500);

        int steps = rrtStar.getStepNo();
        assertEquals(504, steps, 10);
        int nodes = 0;
        for (var n : rrtStar.getNodes()) {
            nodes++;
        }
        assertEquals(500, nodes, 10);
        Path bestPath = rrtStar.getBestPath();
        assertEquals(17.5, bestPath.get_dist(), 2);
    }

}
