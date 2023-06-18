package org.team100.lib.simulation;

import org.team100.lib.reference.Reference;
import org.team100.lib.reference.examples.TrapezoidalReference1D;

import edu.wpi.first.math.numbers.N2;

public class Trajectory extends Scenario {
    private final Reference<N2> reference = new TrapezoidalReference1D();

    Reference<N2> reference() {
        return reference;
    }

    @Override
    String label() {
        return "TRAJECTORY";
    }

    // @Test
    public void test() {
        Loop loop = new Loop(this);
        loop.run();
    }

}
