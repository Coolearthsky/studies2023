package org.team100.lib.estimator;

import java.util.function.BiFunction;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalIntegration;

/**
 * Solves the initial value problem with random vectors via integration.
 */
public class IntegratingPredictor<States extends Num, Inputs extends Num> {
    private final BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f;

    public IntegratingPredictor(BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f) {
        this.f = f;
    }

    /**
     * Project the model into the future with a new control input u.
     *
     * @param x   Previous estimate.
     * @param u   New control input from controller, note this is not uncertain
     * @param dtS Timestep for prediction, seconds.
     */
    public RandomVector<States> predict(RandomVector<States> x0, Matrix<Inputs, N1> u, double dtS) {
        Matrix<States, N1> x0x = x0.x;
        Matrix<States, States> x0P = x0.P;
        Matrix<States, States> xnewP = x0P.copy();
        xnewP.fill(0);
        Matrix<States, N1> xNewX = NumericalIntegration.rk4(f, x0x, u, dtS);
        // the covariance propagates just like the mean.
        for (int col = 0; col < x0P.getNumCols(); ++col) {
            Matrix<States, N1> Pcol = NumericalIntegration.rk4(f, x0P.extractColumnVector(col), u, dtS);
            xnewP.setColumn(col, Pcol);
        }
        RandomVector<States> xNew = new RandomVector<States>(xNewX, xnewP);
        return xNew;
    }

}
