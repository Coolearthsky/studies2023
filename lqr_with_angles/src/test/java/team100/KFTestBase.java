package team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

public abstract class KFTestBase {

    static final double kDelta = 0.001;
    static final double kDt = 0.02;

    // MODEL

    // state: x is (angle (rad), angular_velocity (rad/s))
    final Nat<N2> states = Nat.N2();
    // control input: u is (volts)
    final Nat<N1> inputs = Nat.N1();
    // measurement output: y is (angle)
    final Nat<N1> outputs = Nat.N1();
    // A: angle derivative is velocity
    final Matrix<N2, N2> A = Matrix.mat(states, states)
            .fill(0, 1, //
                    0, 0);
    // B: control input adds velocity
    final Matrix<N2, N1> B = Matrix.mat(states, inputs)
            .fill(0, //
                    1);
    // C: measurement output is angle
    final Matrix<N1, N2> C = Matrix.mat(outputs, states)
            .fill(1, 0);
    // D: control input does not affect measurement directly
    final Matrix<N1, N1> D = Matrix.mat(outputs, inputs)
            .fill(0);
    final LinearSystem<N2, N1, N1> plant = new LinearSystem<>(A, B, C, D);

    // OBSERVER
    // observers are in subclasses; they don't share a superclass.  :-(

    // Q: state stdev
    final Vector<N2> Q = VecBuilder.fill(0.015, 0.17);
    // R: measurement stdev
    final Vector<N1> R = VecBuilder.fill(0.01);

    // CONTROLLER

    final Vector<N2> stateTolerance = VecBuilder
            .fill(0.01, // angle (rad)
                    0.2); // velocity (rad/s)
    final Vector<N1> controlTolerance = VecBuilder
            .fill(12.0); // output (volts)

    final LinearQuadraticRegulator<N2, N1, N1> controller;

    public KFTestBase() {
        controller = newController();
    }

    abstract LinearQuadraticRegulator<N2, N1, N1> newController();

    @Test
    public void testController() {
        // conK: controller gain
        Matrix<N1, N2> conK = controller.getK();
        assertEquals(572.773, conK.get(0, 0), kDelta);
        assertEquals(44.336, conK.get(0, 1), kDelta);

        // conR: controller R aka setpoint, initially zero
        Matrix<N2, N1> conR = controller.getR();
        assertEquals(0, conR.get(0, 0), kDelta);
        assertEquals(0, conR.get(1, 0), kDelta);
    }

}
