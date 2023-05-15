package team100;

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
 * The model for this test is a 1-DOF arm without gravity.
 * 
 * state: (angle, angular velocity) <= the zeroth element here wraps
 * measurement: angle
 * output: torque, i guess?
 */
public class AngleLQR extends LinearQuadraticRegulator<N2, N1, N1> {

    private Matrix<N1, N1> actualU = VecBuilder.fill(0);
    
    public AngleLQR(
            LinearSystem<N2, N1, N1> plant,
            Vector<N2> qelms,
            Vector<N1> relms,
            double dtSeconds) {
        super(plant, qelms, relms, dtSeconds);
    }

    /**
     * Wrap the angle, which is in row zero.
     */
    @Override
    public Matrix<N1, N1> calculate(Matrix<N2, N1> x) {
        Matrix<N2, N1> error = getR().minus(x);
        error.set(0, 0, MathUtil.angleModulus(error.get(0, 0)));
        actualU =  getK().times(error);
        return actualU;
    }

    @Override
    public Matrix<N1, N1> getU() {
        return actualU;
    }

    @Override
    public double getU(int row) {
        return getU().get(row,0);
    }
}