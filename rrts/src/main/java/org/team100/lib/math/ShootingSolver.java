package org.team100.lib.math;

import java.util.Arrays;
import java.util.function.BiFunction;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalIntegration;

/** Shooting method */
public class ShootingSolver<States extends Num, Inputs extends Num> {
    public class Solution {
        public Matrix<Inputs, N1> u;
        public double dt;

        public Solution(Matrix<Inputs, N1> u, double dt) {
            this.u = u;
            this.dt = dt;
        }

    }

    private static double TOLERANCE = 0.01;
    Matrix<Inputs, N1> maxU;
    double maxDt;
    int maxSteps;

    public ShootingSolver(Matrix<Inputs, N1> maxU, double maxDt, int maxSteps) {
        this.maxU = maxU;
        this.maxDt = maxDt;
        this.maxSteps = maxSteps;
    }

    public Solution solve(Nat<States> states, Nat<Inputs> inputs,
            BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f,
            Matrix<States, N1> x1,
            Matrix<States, N1> x2) {
        return solve(states, inputs, f, x1, x2, this.maxU.times(-1), this.maxU, 0, this.maxDt, 0);
        // if (!possible(states, inputs, f, x1, x2))
        // return null;

        // double dt = maxDt;
        // Matrix<Inputs, N1> u = this.maxU.times(0);
        // // Matrix<States, N1> minX2 = NumericalIntegration.rk4(f, x1, u, dt);
        // // this is definitely wrong
        // return new Solution(u, dt);
    }

    static <U extends Num, V extends Num> String matStr(Matrix<U, V> mat) {
        String result = "";
        for (double d : mat.getData()) {
            result += String.format("%5.3f ", d);
        }
        return result;
    }

    public Solution solve(Nat<States> states, Nat<Inputs> inputs,
            BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f,
            Matrix<States, N1> x1,
            Matrix<States, N1> x2,
            Matrix<Inputs, N1> minU,
            Matrix<Inputs, N1> maxU,
            double minDt,
            double maxDt,
            int index) {
        System.out.printf("solve minU %s maxU %s minDt %5.3f maxDt %5.3f index %d\n",
                matStr(minU), matStr(maxU), minDt, maxDt, index);
        if (index > maxSteps) {
            System.out.println("index too high");
            return null;
        }
        // choose the midpoint of time and control axes
        double midDt = (minDt + maxDt) / 2;
        Matrix<Inputs, N1> midU = minU.plus(maxU).div(2);
        // find the resulting state
        Matrix<States, N1> midX2 = NumericalIntegration.rk4(f, x1, midU, midDt);
        if (near(x2, midX2)) {
            System.out.println("near midpoint");
            return new Solution(midU, midDt);
        }
        int axis = index % (inputs.getNum() + 1);
        System.out.printf("axis %d\n", axis);
        if (axis == 0) {
            // use the time axis
            Matrix<States, N1> minX2 = NumericalIntegration.rk4(f, x1, midU, minDt);
            double x2minX2 = x2.minus(minX2).normF();
            if (x2minX2 < TOLERANCE) {
                System.out.println("near min dt");
                return new Solution(midU, minDt);
            }
            Matrix<States, N1> maxX2 = NumericalIntegration.rk4(f, x1, midU, maxDt);
            double x2maxX2 = x2.minus(maxX2).normF();
            if (x2maxX2 < TOLERANCE) {
                System.out.println("near max dt");
                return new Solution(midU, maxDt);
            }
            double minX2maxX2 = minX2.minus(maxX2).normF();
            System.out.printf("minX2 %s maxX2 %s x2 %s\n", matStr(minX2), matStr(maxX2), matStr(x2));
            if (minX2maxX2 < TOLERANCE) {
                System.out.println("min dt near max");
                // just try the next axis?
                return solve(states, inputs, f, x1, x2, minU, maxU, minDt, maxDt, index + 1);
            }
            if (x2minX2 < x2maxX2) {
                System.out.println("look lower dt");
                return solve(states, inputs, f, x1, x2, minU, maxU, minDt, midDt, index + 1);
            } else {
                System.out.println("look higher dt");
                return solve(states, inputs, f, x1, x2, minU, maxU, midDt, maxDt, index + 1);
            }
        } else {
            // use one of the u axes; use the midpoint for all the control values
            // except one
            Matrix<Inputs, N1> minU2 = midU.copy();
            minU2.set(axis - 1, 0, minU.get(axis - 1, 0));
            Matrix<States, N1> minX2 = NumericalIntegration.rk4(f, x1, minU2, midDt);
            double x2minX2 = x2.minus(minX2).normF();
            if (x2minX2 < TOLERANCE) {
                System.out.println("near min u");
                return new Solution(minU2, midDt);
            }
            Matrix<Inputs, N1> maxU2 = midU.copy();
            maxU2.set(axis - 1, 0, maxU.get(axis - 1, 0));
            Matrix<States, N1> maxX2 = NumericalIntegration.rk4(f, x1, maxU2, midDt);
            double x2maxX2 = x2.minus(maxX2).normF();
            if (x2maxX2 < TOLERANCE) {
                System.out.println("near max u");
                return new Solution(maxU2, midDt);
            }
            double minX2maxX2 = minX2.minus(maxX2).normF();
            System.out.printf("minX2 %s maxX2 %s x2 %s\n", matStr(minX2), matStr(maxX2), matStr(x2));
            if (minX2maxX2 < TOLERANCE) {
                System.out.println("min u near max");
                // just try the next axis?
                return solve(states, inputs, f, x1, x2, minU, maxU, minDt, maxDt, index + 1);
            }
            if (x2minX2 < x2maxX2) {
                System.out.println("look lower u");
                return solve(states, inputs, f, x1, x2, minU2, midU, minDt, maxDt, index + 1);
            } else {
                System.out.println("look higher u");
                return solve(states, inputs, f, x1, x2, midU, maxU2, minDt, maxDt, index + 1);
            }
        }
    }

    public boolean near(Matrix<States, N1> x1, Matrix<States, N1> x2) {
        return x1.isIdentical(x2, TOLERANCE);
    }

    /**
     * For the one-dimensional control case, return true if the target state is
     * within the triangle swept by u and dt.
     * 
     * TODO: this is definitely wrong for higher dimensionality, it tests the
     * surface
     * from minimum U (in all dimensions) to maximum U (in all dimensions), i.e. a
     * plane cutting through the hypervolume that we should really test.
     * 
     * The full range of u and dt values paints a surface that includes x1, minX2,
     * and maxX2, like a wavy triangle. In the 2d case it's exactly a triangle and
     * x2 is in the same plane, but i'd like this to work for 4d.
     * 
     * This function assumes the waviness is small relative to the tolerance, and
     * checks "inside" via the angles to the vertices.
     * 
     * Note the triangle may be degenerate: imagine a motionless start, pushed
     * by positive or negative u -- the min, start, and max will all be on the same
     * line.
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
     * Inside test for one input dimension, which together with dt forms a
     * two-dimensional space of possible results.
     * 
     * @param x1    start
     * @param x2    goal
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

        double angleSum = angleSum(x2x1, x2minX2, x2maxX2);

        // if the point is near the triangle then the angles should add to 2pi.
        // there's no way the angles could possibly be *more* than 2pi.
        if (angleSum > 2.0 * Math.PI)
            throw new RuntimeException();
        if (2.0 * Math.PI - angleSum < TOLERANCE)
            return true;
        return false;
    }

    /**
     * For the one-control-dimension case, the two-dimensional result can be
     * approximated by a triangle; this calculates the sum of the angles from the
     * target point to each triangle vertex.
     */
    public double angleSum(Matrix<States, N1> x2x1,
            Matrix<States, N1> x2minX2,
            Matrix<States, N1> x2maxX2) {
        double x2x1_x2minX2 = angleRad(x2x1, x2minX2);
        double x2x1_x2maxX2 = angleRad(x2x1, x2maxX2);
        double x2minX2_x2maxX2 = angleRad(x2minX2, x2maxX2);
        double angleSum = x2x1_x2minX2 + x2x1_x2maxX2 + x2minX2_x2maxX2;
        return angleSum;
    }

    /** Return the angle between the vectors, using the dot product. */
    public double angleRad(Matrix<States, N1> x, Matrix<States, N1> y) {
        double dot = x.transpose().times(y).get(0, 0);
        double xNorm = x.normF();
        double yNorm = y.normF();
        double angleRad = Math.acos(dot / (xNorm * yNorm));
        return angleRad;
    }

}
