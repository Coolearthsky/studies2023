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

    Matrix<Inputs, N1> maxU;
    double maxDt;

    public ShootingSolver(Matrix<Inputs, N1> maxU, double maxDt) {
        this.maxU = maxU;
        this.maxDt = maxDt;
    }

    /**
     * Returns the constant u value to get from x1 to x2 in time dt, or null if the
     * path is infeasible.
     */
    public Solution shoot(Nat<States> states, Nat<Inputs> inputs,
            BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f,
            Matrix<States, N1> x1,
            Matrix<States, N1> x2) {
        double dt = maxDt;
        Matrix<Inputs, N1> maxU = this.maxU;
        Matrix<States, N1> maxX2 = NumericalIntegration.rk4(f, x1, maxU, dt);
        Matrix<Inputs, N1> minU = this.maxU.times(-1);
        Matrix<States, N1> minX2 = NumericalIntegration.rk4(f, x1, minU, dt);
        // The full range of U values paints a surface that includes x1,minX2, and maxX2,
        // in the simplest case it's a like a wavy triangle.
        // in the 2d case it's exactly a triangle and x2 is in the same plane,
        // but i'd like this to work for 4d.

        // first check to see if x2 is in the plane of the "triangle".

        // then check if x2 is inside the triangle.

        // both these checks could be combined by looking at the angles...

        

    }

}
