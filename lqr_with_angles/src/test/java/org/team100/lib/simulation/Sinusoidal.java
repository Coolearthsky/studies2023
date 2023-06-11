package org.team100.lib.simulation;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;

public  class Sinusoidal extends Scenario {
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

   // @Test
    public void test() {
        init();
        execute();
    }
}