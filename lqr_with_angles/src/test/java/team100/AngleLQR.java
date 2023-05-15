package team100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;

/**
 * For testing LQR with angle wrapping. Wraps an angle in the zeroth row of the
 * state vector.
 */
public class AngleLQR<States extends Num, Inputs extends Num, Outputs extends Num>
        extends LinearQuadraticRegulator<States, Inputs, Outputs> {
    public AngleLQR(
            LinearSystem<States, Inputs, Outputs> plant,
            Vector<States> qelms,
            Vector<Inputs> relms,
            double dtSeconds) {
        super(plant, qelms, relms, dtSeconds);
    }

    /**
     * Wrap the error angle, which is in row zero.
     */
    @Override
    public Matrix<Inputs, N1> calculate(Matrix<States, N1> x) {
        Matrix<States, N1> error = getR().minus(x);
        error.set(0, 0, MathUtil.angleModulus(error.get(0, 0)));
        return getK().times(error);
    }

    /** Don't use this, use the value returned by calculate(). */
    @Override
    public Matrix<Inputs, N1> getU() {
        throw new UnsupportedOperationException();
    }

    /** Don't use this, use the value returned by calculate(). */
    @Override
    public double getU(int row) {
        throw new UnsupportedOperationException();
    }
}