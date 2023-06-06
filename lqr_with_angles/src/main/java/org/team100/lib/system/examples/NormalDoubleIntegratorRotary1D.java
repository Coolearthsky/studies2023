package org.team100.lib.system.examples;

import org.team100.lib.system.Sensor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Specifies stdev's. */
public class NormalDoubleIntegratorRotary1D extends DoubleIntegratorRotary1D {
    public class NormalFullSensor extends FullSensor {
        public Matrix<N2, N1> stdev() {
            return VecBuilder.fill(0.01, 0.1);
        }
    }

    public class NormalPositionSensor extends PositionSensor {
        public Matrix<N1, N1> stdev() {
            return VecBuilder.fill(0.01);
        }
    }

    public class NormalVelocitySensor extends VelocitySensor {
        public Matrix<N1, N1> stdev() {
            return VecBuilder.fill(0.1);
        }
    }

    public Matrix<N2, N1> stdev() {
        return VecBuilder.fill(0.015, 0.17);
    }

    public Sensor<N2, N1, N1> newPosition() {
        return new NormalPositionSensor();
    }

    public Sensor<N2, N1, N1> newVelocity() {
        return new NormalVelocitySensor();
    }

    public Sensor<N2, N1, N2> newFull() {
        return new NormalFullSensor();
    }
}
