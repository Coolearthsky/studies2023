package org.team100.lib.prrts;

import org.junit.jupiter.api.Test;

import edu.unc.robotics.prrts.PRRTStar;
import edu.unc.robotics.prrts.example.arena.HolonomicArena;

public class TestCorruption {
    @Test
    public void testThreads() {
        for (int i = 0; i < 100; ++i) {

            final HolonomicArena arena = new HolonomicArena();
            double[] init = { 7.0, 1.0 };

            final PRRTStar rrtStar = new PRRTStar(arena, arena, init);

            Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
            Thread.currentThread().getThreadGroup().setMaxPriority(Thread.MIN_PRIORITY);

            rrtStar.runForDurationMS(2, 6.0, 20);
        }

    }

}
