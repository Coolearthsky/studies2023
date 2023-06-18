package org.team100.lib.simulation;

import org.junit.jupiter.api.Test;
import org.team100.lib.reference.Reference;
import org.team100.lib.reference.examples.SinusoidalReference1D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N2;

public  class Sinusoidal extends Scenario {
    private final Reference<N2> reference = new SinusoidalReference1D();

    Reference<N2> reference() {
        return reference;
    }

    double position(double timeSec) {
        return MathUtil.angleModulus(Math.cos(timeSec));
    }

    double velocity(double timeSec) {
        return -1.0 * Math.sin(timeSec);
    }

    double acceleration(double timeSec) {
        return -1.0 * Math.cos(timeSec);
    }

    String label() {
        return "SINUSOIDAL";
    }

    //@Test
    public void test() {
        Loop loop = new Loop(this);
        loop.run();
    }
}