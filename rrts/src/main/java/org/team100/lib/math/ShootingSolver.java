package org.team100.lib.math;

import java.util.function.BiFunction;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalIntegration;

/** Shooting method */
public class ShootingSolver<States extends Num, Inputs extends Num> {
    public class Solution {
        public Matrix<Inputs, N1> u;
        public double dt;
    }

    private static double TOLERANCE = 0.01;
    Matrix<Inputs, N1> maxU;
    double maxDt;

    public ShootingSolver(Matrix<Inputs, N1> maxU, double maxDt) {
        this.maxU = maxU;
        this.maxDt = maxDt;
    }

    /**
     * Returns the constant u value to get from x1 to x2 in time dt, or null if the
     * path is infeasible.
     * 
     * The full range of u and dt values paints a surface that includes x1, minX2,
     * and maxX2, like a wavy triangle. In the 2d case it's exactly a triangle and
     * x2 is in the same plane, but i'd like this to work for 4d.
     * 
     * This function assumes the waviness is small relative to the tolerance, and
     * checks "inside" via the angles to the vertices.
     * 
     * 
     * @param x1 source state
     * @param x2 target state
     */
    public boolean possible(Nat<States> states, Nat<Inputs> inputs,
            BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f,
            Matrix<States, N1> x1,
            Matrix<States, N1> x2) {

        double dt = maxDt;
        Matrix<Inputs, N1> minU = this.maxU.times(-1);
        Matrix<States, N1> minX2 = NumericalIntegration.rk4(f, x1, minU, dt);

        Matrix<Inputs, N1> maxU = this.maxU;
        Matrix<States, N1> maxX2 = NumericalIntegration.rk4(f, x1, maxU, dt);

        return inside(x1, x2, minX2, maxX2);
    }

    /**
     * @param x1 start
     * @param x2 goal
     * @param minX2 reached by min u
     * @param maxX2 reached by max u
     */
    public boolean inside(
            Matrix<States, N1> x1,
            Matrix<States, N1> x2,
            Matrix<States, N1> minX2,
            Matrix<States, N1> maxX2) {

        Matrix<States, N1> x2x1 = x2.minus(x1);
        // the start and end are about the same
        if (x2x1.normF() < TOLERANCE)
            return true;

        Matrix<States, N1> x2minX2 = x2.minus(minX2);
        // the end is reached by min u
        if (x2minX2.normF() < TOLERANCE)
            return true;

        Matrix<States, N1> x2maxX2 = x2.minus(maxX2);
        // the end is reached by max u
        if (x2maxX2.normF() < TOLERANCE)
            return true;

        // there are three angles between the three vectors
        double x2x1_x2minX2 = angleRad(x2x1, x2minX2);
        double x2x1_x2maxX2 = angleRad(x2x1, x2maxX2);
        double x2minX2_x2maxX2 = angleRad(x2minX2, x2maxX2);
        double angleSum = x2x1_x2minX2 + x2x1_x2maxX2 + x2minX2_x2maxX2;

        // if the point is near the triangle then the angles should add to 2pi.
        // there's no way the angles could possibly be *more* than 2pi.
        if (angleSum > 2.0 * Math.PI)
            throw new RuntimeException();
        if (2.0 * Math.PI - angleSum < TOLERANCE)
            return true;
        return false;
    }

    public double angleRad(Matrix<States, N1> x, Matrix<States, N1> y) {
        double dot = x.transpose().times(y).get(0, 0);
        double xNorm = x.normF();
        double yNorm = y.normF();
        double angleRad = Math.acos(dot / (xNorm * yNorm));
        return angleRad;
    }

}
