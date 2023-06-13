package org.team100.lib.simulation;

import edu.wpi.first.math.MathUtil;

public class ConstantAcceleration extends Scenario {
    private static final double kAcceleration = 1.0;

    double position(double timeSec) {
        return MathUtil.angleModulus(Math.pow(timeSec, 2) / 2);
    }

    double velocity(double timeSec) {
        return kAcceleration * timeSec;
    }

    double acceleration(double timeSec) {
        return kAcceleration;
    }

    String label() {
        return "CONSTANT ACCELERATION";
    }

    // @Test
    public void test() {
        Loop loop = new Loop(this);
        loop.run();
    }
}