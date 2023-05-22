package team100.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

/**
 * For testing LQR with angle wrapping.
 * 
 * Has an LQR inside which is pretty much only used to calculate K.
 * 
 * The model for this test is a 1-DOF arm without gravity.
 * 
 * state: (angle, angular velocity) <= the zeroth element here wraps
 * measurement: angle
 * output: torque, i guess?
 */
public class AngleController {

    private final LinearQuadraticRegulator<N2, N1, N2> lqr;
    private Matrix<N2, N1> actualR = VecBuilder.fill(0, 0);
    private Matrix<N1, N1> actualU = VecBuilder.fill(0);

    public AngleController(
            LinearSystem<N2, N1, N2> plant,
            Vector<N2> qelms,
            Vector<N1> relms,
            double dtSeconds) {
        lqr = new LinearQuadraticRegulator<N2, N1, N2>(plant, qelms, relms, dtSeconds);
    }

    /**
     * Wrap the angle, which is in row zero.
     */
    public Matrix<N1, N1> calculate(Matrix<N2, N1> x) {
        Matrix<N2, N1> error = getR().minus(x);
        error.set(0, 0, MathUtil.angleModulus(error.get(0, 0)));
        actualU = getK().times(error);
        return actualU;
    }

    /**
     * Output is not aware of actuator limits, that needs to be
     * handled by the caller.
     */
    public Matrix<N1, N1> getU() {
        return actualU;
    }
    
    /**
     * Output is not aware of actuator limits, that needs to be
     * handled by the caller.
     */
    public double getU(int row) {
        return getU().get(row, 0);
    }

    public Matrix<N1, N2> getK() {
        return lqr.getK();
    }

    public Matrix<N2, N1> getR() {
        return actualR;
    }

    public void reset() {
        lqr.reset();
    }

    public Matrix<N1, N1> calculate(Matrix<N2, N1> x, Matrix<N2, N1> nextR) {
        actualR = nextR;
        return calculate(x);
    }
}