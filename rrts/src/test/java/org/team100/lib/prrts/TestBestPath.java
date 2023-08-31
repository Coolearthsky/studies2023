package org.team100.lib.prrts;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.planner.Runner;
import org.team100.lib.rrt.RRTStar;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;

import edu.unc.robotics.prrts.example.arena.HolonomicArena;

@SuppressWarnings("unused")
public class TestBestPath {

    /** Cribbed from ArenaFrame */
    @Test
    public void testTime() {
        final HolonomicArena arena = new HolonomicArena(6);
        final RRTStar<HolonomicArena> worker = new RRTStar<>(arena, new Sample(arena), 6);
        final Runner rrtStar = new Runner(worker);

        rrtStar.runForDurationMS(20);

        int steps = rrtStar.getStepNo();
        assertEquals(2400, steps, 1200); 

        int nodes = 0;
        for (var n : rrtStar.getNodesA()) {
            nodes++;
        }
        for (var n : rrtStar.getNodesB()) {
            nodes++;
        }
        assertEquals(2400, nodes, 1200);
        Path bestPath = rrtStar.getBestPath();
        assertEquals(16.5, bestPath.getDistance(), 1.5);
    }

    @Test
    public void testSteps() {
        final HolonomicArena arena = new HolonomicArena(6);
        final RRTStar<HolonomicArena> worker = new RRTStar<>(arena, new Sample(arena), 6);
        final Runner rrtStar = new Runner(worker);

        rrtStar.runSamples(500);

        int steps = rrtStar.getStepNo();
        assertEquals(504, steps, 10);
        int nodes = 0;
        for (var n : rrtStar.getNodesA()) {
            nodes++;
        }
        for (var n : rrtStar.getNodesB()) {
            nodes++;
        }
        assertEquals(500, nodes, 10);
        Path bestPath = rrtStar.getBestPath();
        assertEquals(17.5, bestPath.getDistance(), 2);
    }

}
