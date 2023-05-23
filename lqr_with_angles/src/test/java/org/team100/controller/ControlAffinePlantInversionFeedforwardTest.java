package org.team100.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ControlAffinePlantInversionFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ControlAffinePlantInversionFeedforwardTest {
    private static final double kDelta = 0.0001;

    /**
     * xdot = f(x,u)
     * pdot = v
     * vdot = u
     * 
     * so the u jacobian should just be [0, 1]
     */
    Matrix<N2, N1> newton1d(Matrix<N2, N1> xmat, Matrix<N1, N1> umat) {
        //double p = xmat.get(0, 0);
        double v = xmat.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u;
        return VecBuilder.fill(pdot, vdot);
    }

    /**
     * xdot = f(x,u)
     * pdot = v
     * vdot = u - cos(p)
     * 
     * so vdot itself depends on p but it is still linear in u.
     */
    Matrix<N2, N1> pendulum(Matrix<N2, N1> xmat, Matrix<N1, N1> umat) {
        double p = xmat.get(0, 0);
        double v = xmat.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u - Math.cos(p);
        return VecBuilder.fill(pdot, vdot);
    }

    @Test
    public void testNewton() {
        // the model is trivial
        // the r and nextR v differ by 1
        // so u should be 1/dt or 50.
        ControlAffinePlantInversionFeedforward<N2, N1> feedforward = new ControlAffinePlantInversionFeedforward<N2, N1>(
                Nat.N2(), Nat.N1(), this::newton1d, 0.02);
        // position does not matter here.
        Matrix<N2, N1> r = VecBuilder.fill(0, 2);
        Matrix<N2, N1> nextR = VecBuilder.fill(1, 3);
        Matrix<N1, N1> uff = feedforward.calculate(r, nextR);
        assertEquals(50, uff.get(0, 0), kDelta);
    }

    @Test
    public void testPendulum() {
        // pendulum model vdot includes position dependence
        // the r and nextR differ by 1
        // so u should be 1/dt or 50.
        ControlAffinePlantInversionFeedforward<N2, N1> feedforward = new ControlAffinePlantInversionFeedforward<N2, N1>(
                Nat.N2(), Nat.N1(), this::pendulum, 0.02);
        {
            // r position zero means 90 degrees which means maximum gravity
            // note nextR is irrelevant for the constant part of ff
            Matrix<N2, N1> r = VecBuilder.fill(0, 0);
            Matrix<N2, N1> nextR = VecBuilder.fill(1000, 0);
            Matrix<N1, N1> uff = feedforward.calculate(r, nextR);
            assertEquals(1, uff.get(0, 0), kDelta);
        }
        {
            // it's easy to construct nonsensical trajectories
            // this one speeds up without moving, so the ff obeys the
            // speeding-up part.
            Matrix<N2, N1> r = VecBuilder.fill(Math.PI / 2, 0);
            Matrix<N2, N1> nextR = VecBuilder.fill(Math.PI / 2, 1);
            Matrix<N1, N1> uff = feedforward.calculate(r, nextR);
            assertEquals(50, uff.get(0, 0), kDelta);
        }
    }

}
