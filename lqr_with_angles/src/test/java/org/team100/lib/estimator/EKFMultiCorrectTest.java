package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.RandomVector;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;
import org.team100.lib.system.examples.NormalDoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Illustrates multiple measurement sources. */
public class EKFMultiCorrectTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    private RandomVector<N2> ypos(double yd) {

        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, yd); // position
        
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.1); // TODO: pass variance somehow
        yP.set(1, 1, 1e9); // velocity gets "don't know" variance
        return new RandomVector<>(yx, yP);



        //return new RandomVector<>(VecBuilder.fill(yd),VecBuilder.fill(0.1));
    }

    private RandomVector<N2> yvel(double yd) {

        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(1, 0, yd); // velocity
        
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 1e9); // position gets "don't know" variance
        yP.set(1, 1, 0.1); // TODO: pass variance somehow
        return new RandomVector<>(yx, yP);

       // return new RandomVector<>(VecBuilder.fill(yd),VecBuilder.fill(0.1));
    }

    @Test
    public void testMultipleSensors() {
        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        IntegratingPredictor<N2, N1> predictor = new IntegratingPredictor<>();
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>();

        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, predictor, pointEstimator, pooling);

        Matrix<N2, N2> p = new Matrix<>(Nat.N2(),Nat.N2());
        p.set(0,0,0.1);
        p.set(1,1,0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        Matrix<N1, N1> u = VecBuilder.fill(-12);

        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, ypos(-3.134), system.position());
        xhat = estimator.correct(xhat, yvel(-0.240), system.velocity());
        assertEquals(-3.134, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.240, xhat.x.get(1, 0), kDelta);

        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, ypos(-3.141), system.position());
        xhat = estimator.correct(xhat, yvel(-0.480), system.velocity());
        assertEquals(-3.141, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.480, xhat.x.get(1, 0), kDelta);

        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, ypos(3.13), system.position());
        xhat = estimator.correct(xhat, yvel(-0.720), system.velocity());
        assertEquals(3.130, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.x.get(1, 0), kDelta);

        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, ypos(3.113), system.position());
        xhat = estimator.correct(xhat, yvel(-0.960), system.velocity());
        assertEquals(3.113, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.x.get(1, 0), kDelta);
    }
}
