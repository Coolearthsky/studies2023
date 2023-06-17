package org.team100.lib.simulation;

import edu.wpi.first.math.MathUtil;

public class ConstantVelocity extends Scenario {
    private static final double kVelocity = 1.0;

    double position(double timeSec) {
        return MathUtil.angleModulus(kVelocity * timeSec);
    }

    double velocity(double timeSec) {
        return kVelocity;
    }

    double acceleration(double timeSec) {
        return 0;
    }

    String label() {
        return "CONSTANT VELOCITY";
    }

    // @Test
    public void test() {
        Loop loop = new Loop(this);
        loop.run();
    }
}