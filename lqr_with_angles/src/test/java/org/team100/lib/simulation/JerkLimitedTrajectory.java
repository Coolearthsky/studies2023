package org.team100.lib.simulation;

import org.junit.jupiter.api.Test;
import org.team100.lib.reference.Reference;
import org.team100.lib.reference.examples.JerkLimitedProfileReference1D;

import edu.wpi.first.math.numbers.N2;

public class JerkLimitedTrajectory extends Scenario {
    private final Reference<N2> reference = new JerkLimitedProfileReference1D();

    Reference<N2> reference() {
        return reference;
    }

    @Override
    String label() {
        return "TRAJECTORY";
    }

    // turn off because it makes a lot of output
    @Test
    public void test() {
        Loop loop = new Loop(this);
        loop.run();
    }
}
