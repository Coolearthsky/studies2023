package edu.unc.robotics.prrts.example.swingup;

import org.team100.lib.graph.Node;
import org.team100.lib.index.KDModel;
import org.team100.lib.index.KDNearNode;
import org.team100.lib.planner.RobotModel;

import edu.unc.robotics.prrts.example.geom.Obstacle;
import edu.wpi.first.math.DARE;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;

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
 * A = [ 0 1, -sin x1 0]
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

    private static final double POSITION_TOLERANCE = 0.25;
    private static final double VELOCITY_TOLERANCE = 0.25;

    private final double[] _init;
    private final double[] _goal;
    /** same as the paper */
    private static final double[] _min = { -4, -8 };
    private static final double[] _max = { 4, 8 };

    Obstacle[] _obstacles = new Obstacle[] {};

    // Q is x norm coord
    //Vector<N2> qelms = VecBuilder.fill(1, 1);
    // R is u norm coord
  //  Vector<N1> relms = VecBuilder.fill(50);

    //Matrix<N2, N2> Q = StateSpaceUtil.makeCostMatrix(qelms);
//    Matrix<N1, N1> R = StateSpaceUtil.makeCostMatrix(relms);

    Matrix<N2,N2> Q = Matrix.eye(Nat.N2());
    Matrix<N1,N1> R = VecBuilder.fill(50);

    double h = 0.1; // TODO: this is surely wrong. what time interval to use?

    // see pend_rrt.m
    private final double m = 1; // mass kg
    private final double l = 1; // length meter
    private final double b = 0.1; // viscous drag, unit = ?
    private final double _g; // gravity m/s/s

    private int stepNo;
    private double radius;

    public PendulumArena(double[] init, double[] goal, double gravity) {
        _init = init;
        _goal = goal;
        _g = gravity;
        // Matrix<N2, N2> S = getS(new double[] { 0, 0 });

    }

    public Matrix<N2, N2> getA(double[] x) {
        return Matrix.mat(Nat.N2(), Nat.N2()).fill(
                0, 1,
                -_g * Math.cos(x[0]), -b / (m * Math.pow(l, 2)));

    }

    public Matrix<N2, N1> getB() {
        return VecBuilder.fill(0, 1 / (m * Math.pow(l, 2)));
    }

    /**
     * use the WPI LQR lib to calculate S.
     */
    public Matrix<N2, N2> getS(double[] x) {
        Matrix<N2, N2> A = getA(x);
        Matrix<N2, N1> B = getB();
        Pair<Matrix<N2, N2>, Matrix<N2, N1>> discABPair = Discretization.discretizeAB(A, B, 1);
        Matrix<N2, N2> discA = discABPair.getFirst();
        Matrix<N2, N1> discB = discABPair.getSecond();
        return DARE.dare(discA, discB, Q, R);
    }

    // TODO: speed this up
    public Matrix<N1, N2> getK(double[] x) {
        Matrix<N2, N2> A = getA(x);
        Matrix<N2, N1> B = getB();
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
    public double[] getMin() {
        return _min.clone();        
    }

    @Override
    public double[] getMax() {
        return _max.clone();        
    }

    /**
     * the distance metric is dx S dx^T where dx is the vector difference and S is
     * the Riccati solution. the key to this approach is linearizing the model at x
     * for each sample.
     */
    @Override
    public double dist(double[] start, double[] end) {
        Matrix<N2, N2> S = getS(start);
        Matrix<N2, N1> dx = VecBuilder.fill(end[0] - start[0], end[1] - start[1]);
        return dx.transpose().times(S).times(dx).get(0, 0);
    }

    @Override
    public void setStepNo(int stepNo) {
        this.stepNo = stepNo;
    }

    @Override
    public void setRadius(double radius) {
        this.radius = radius;
    }

    /**
     * steer from the near config towards the new config using the real dynamics and
     * LQR full-state control, i.e. find the u value to try to go from the current
     * point to the new point, and then integrate forward to find the new point.
     */
    @Override
    public double[] steer(KDNearNode<Node> x_nearest, double[] newConfig) {
        double[] nearConfig = x_nearest._nearest.getState();

        // see pend_rrt.m
        Matrix<N2, N1> x_near = VecBuilder.fill(nearConfig[0], nearConfig[1]);
        Matrix<N2, N1> x_rand = VecBuilder.fill(newConfig[0], newConfig[1]);
        Matrix<N2, N1> dx = x_near.minus(x_rand);
        Matrix<N1, N2> K = getK(newConfig);
        double u = K.times(-1).times(dx).get(0, 0);
        u = Math.max(-3, u);
        u = Math.min(3, u);
        Matrix<N2, N1> xdot = VecBuilder.fill(
                nearConfig[1],
                (u - b * nearConfig[1] - m * _g * l * Math.sin(nearConfig[0])));
        Matrix<N2, N1> x_new = x_near.plus(xdot.times(h));
        return x_new.getData();
      //  newConfig[0] = x_new.get(0, 0);
       // newConfig[1] = x_new.get(1, 0);
    }

    @Override
    public double[] initial() {
        return _init;
    }

    @Override
    public boolean goal(double[] config) {
        if (Math.abs(config[0] - _goal[0]) > POSITION_TOLERANCE)
            return false;
        if (Math.abs(config[1] - _goal[1]) > VELOCITY_TOLERANCE)
            return false;
        return true;
    }

    /** There aren't really obstacles, but this will come in handy later. */
    @Override
    public boolean clear(double[] config) {
        return true;
    }

    /**
     * This checks for obstacles along the path. between a and b but since there
     * aren't any obstacles.
     */
    @Override
    public boolean link(double[] a, double[] b) {
        return true;
    }

    public Obstacle[] obstacles() {
        return _obstacles;
    }

}
