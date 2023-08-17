package edu.unc.robotics.prrts.example.swingup;

import java.io.IOException;

import edu.unc.robotics.prrts.RobotModel;
import edu.unc.robotics.prrts.example.geom.Obstacle;
import edu.unc.robotics.prrts.kdtree.KDModel;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;

import edu.wpi.first.math.DARE;

/**
 * Single jointed pendulum
 * 
 * Example of non-euclidean space.
 * 
 * x1 = angle from downward
 * x2 = velocity
 * 
 * x1dot = x2
 * x2dot = - g * sin(x1) / l + u
 * 
 * let's say g = l.
 * 
 * A = [ 0 1, -cos x1 0]
 * B = [0, 1]
 * 
 * note since this uses wpimathjni you have to run it in some way that knows
 * where the jni libs are, e.g. using the vscode "simulate robot" button.
 * 
 * The math here follows
 * https://github.com/MahanFathi/LQR-RRTstar
 * http://people.csail.mit.edu/tlp/pdf/2012/ICRA12_1657_FI.pdf
 */
public class PendulumArena implements RobotModel, KDModel {
    /**
     * zeroth dimension is position (radians, down is zero).
     * first dimension is velocity (radians per second).
     */
    private static final int DIMENSIONS = 2;

    /** goal is straight up, motionless. */
    private static final double[] _goal = { Math.PI, 0 };
    /** guessing on the velocity limit */
    private static final double[] _min = { -2 * Math.PI, -2 * Math.PI };
    private static final double[] _max = { 2 * Math.PI, 2 * Math.PI };

    Obstacle[] _obstacles = new Obstacle[] {};

    Vector<N2> qelms = VecBuilder.fill(1.0, 1.0);
    Vector<N1> relms = VecBuilder.fill(1.0);

    Matrix<N2, N2> Q = StateSpaceUtil.makeCostMatrix(qelms);
    Matrix<N1, N1> R = StateSpaceUtil.makeCostMatrix(relms);

    Matrix<N2, N1> B = VecBuilder.fill(0, 1);

    double h = 0.02; // TODO: this is surely wrong. what time interval to use?

    public PendulumArena() {

        Matrix<N2, N2> S = getS(new double[] { 0, 0 });

        System.out.println("==============");
        System.out.println(S);
        System.out.println("==============");

    }

    /**
     * use the WPI LQR lib to calculate S.
     */
    Matrix<N2, N2> getS(double[] x) {
        Matrix<N2, N2> A = Matrix.mat(Nat.N2(), Nat.N2()).fill(0, 1, -1 * Math.cos(x[0]), 0);
        Pair<Matrix<N2, N2>, Matrix<N2, N1>> discABPair = Discretization.discretizeAB(A, B, 1);
        Matrix<N2, N2> discA = discABPair.getFirst();
        Matrix<N2, N1> discB = discABPair.getSecond();
        return DARE.dare(discA, discB, Q, R);
    }

    // TODO: speed this up
    Matrix<N1, N2> getK(double[] x) {
        Matrix<N2, N2> A = Matrix.mat(Nat.N2(), Nat.N2()).fill(0, 1, -1 * Math.cos(x[0]), 0);
        Pair<Matrix<N2, N2>, Matrix<N2, N1>> discABPair = Discretization.discretizeAB(A, B, 1);
        Matrix<N2, N2> discA = discABPair.getFirst();
        Matrix<N2, N1> discB = discABPair.getSecond();
        Matrix<N2, N2> S = DARE.dare(discA, discB, Q, R);
        return discB
                .transpose()
                .times(S)
                .times(discB)
                .plus(R)
                .solve(discB.transpose().times(S).times(discA));
    }

    @Override
    public int dimensions() {
        return DIMENSIONS;
    }

    @Override
    public void getBounds(double[] min, double[] max) {
        System.arraycopy(_min, 0, min, 0, DIMENSIONS);
        System.arraycopy(_max, 0, max, 0, DIMENSIONS);
    }

    /**
     * the distance metric is dx S dx^T where dx is the vector difference and S is
     * the Riccati solution. the key to this approach is linearizing the model at x
     * for each sample.
     */
    @Override
    public double dist(double[] start, double[] end) {
        Matrix<N2, N2> S = getS(start);
        Matrix<N2, N1> dx =  VecBuilder.fill(end[0] - start[0], end[1] - start[1]);
        return dx.transpose().times(S).times(dx).get(0, 0);
    }

    /**
     * steer from the near config towards the new config using the real dynamics
     */
    @Override
    public void steer(double[] nearConfig, double[] newConfig, double dist) {
        Matrix<N2,N1> x_near = VecBuilder.fill(nearConfig[0],nearConfig[1]);
        Matrix<N2,N1> x_rand = VecBuilder.fill(newConfig[0],newConfig[1]);
        Matrix<N2, N1> dx =  x_near.minus(x_rand);
        Matrix<N1,N2> K = getK(newConfig);
        double u = K.times(-1).times(dx).get(0,0);
        Matrix<N2,N1> xdot  = VecBuilder.fill(nearConfig[1], (u - Math.sin(nearConfig[0])));
        Matrix<N2, N1> x_new = x_near.plus(xdot.times(h));
        newConfig[0] = x_new.get(0,0);
        newConfig[1] = x_new.get(1,0);
    }

    @Override
    public boolean goal(double[] config) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'goal'");
    }

    @Override
    public boolean clear(double[] config) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'clear'");
    }

    @Override
    public boolean link(double[] a, double[] b) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'link'");
    }

    public Obstacle[] obstacles() {
        return _obstacles;
    }

}
