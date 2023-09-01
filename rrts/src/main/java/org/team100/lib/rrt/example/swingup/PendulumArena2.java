package org.team100.lib.rrt.example.swingup;
import org.team100.lib.example.Arena;
import org.team100.lib.geom.Obstacle;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNearNode;

/**
 * 
 * This version does not use LQR math.
 * 
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
 * the general idea is
 */
public class PendulumArena2 implements Arena {
    /**
     * zeroth dimension is position (radians, down is zero).
     * first dimension is velocity (radians per second).
     */
    private static final int DIMENSIONS = 2;

    private static final double MIN_U = -0.5;
    private static final double MAX_U = 0.5;
    private static final double MIN_DT = 0.01;
    private static final double MAX_DT = 0.25;

    private static final double POSITION_TOLERANCE = 0.25;
    private static final double VELOCITY_TOLERANCE = 0.25;

    private final double[] _init;
    private final double[] _goal;
    /** same as the paper */
    private static final double[] _min = { -4, -8 };
    private static final double[] _max = { 4, 8 };

    Obstacle[] _obstacles = new Obstacle[] {};

    double h = 0.1; // TODO: this is surely wrong. what time interval to use?

    // see pend_rrt.m
    // private final double m = 1; // mass kg
    private final double l = 1; // length meter
    // private final double b = 0.1; // viscous drag, unit = ?
    private final double _g; // gravity m/s/s

    // private int stepNo;
    // private double radius;
    private boolean timeForward = true;


    public PendulumArena2(double[] init, double[] goal, double gravity) {
        _init = init;
        _goal = goal;
        _g = gravity;

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
     * try to make this return time.
     * 
     * x1dot = x2
     * x2dot = something + u
     * 
     * 
     */
    @Override
    public double dist(double[] start, double[] end) {

        double x_start1 = start[0];
        double x_start2 = start[1];
        double x_end1 = end[0];
        double x_end2 = end[1];
        // TODO: what should x1dot actually be?
        double x1dot = (x_start2 + x_end2)/2;
        double dx1 = x_end1 - x_start1;
        double dt = dx1 / x1dot;

        // System.out.printf("dist from [%5.3f %5.3f] to [%5.3f %5.3f] dt %5.3f u %5.3f\n",
                //  x_start1, x_start2, x_end1, x_end2, dt, 0.0);
        return dt;


        // double dist = 0;
        // for (int i = 0; i < DIMENSIONS; i += 2) {
        //     double dx = start[i] - end[i];
        //     double dy = start[i + 1] - end[i + 1];
        //     dist += dx * dx + dy * dy;
        // }
        // return Math.sqrt(dist);
    }

    @Override
    public void setStepNo(int stepNo) {
        // this.stepNo = stepNo;
    }

    // for now this means the direction of time
    // TODO: a different way to do that
    @Override
    public void setRadius(double radius) {
        timeForward = (radius > 0);
        // this.radius = radius;
    }

    /**
     * steer from the near config towards the new config using the real dynamics and
     * bang-bang control, i.e. find the u value to try to go from the current
     * point to the new point, and then integrate forward to find the new point.
     * 
     * Returns null if there is no easy way to get to the goal state.
     * 
     * @param x_nearest starting state
     * @param x_rand    goal state
     * @return x_new a feasible state
     */
    @Override
    public double[] steer(KDNearNode<Node> x_nearest, double[] x_rand) {

        if (x_nearest._nearest == null) {
            return null;
        }
        double[] x_nearest_state = x_nearest._nearest.getState();
        double x_nearest1 = x_nearest_state[0];
        double x_nearest2 = x_nearest_state[1];

        double x_rand1 = x_rand[0];
        double x_rand2 = x_rand[1];

        // from system dynamics
        double x1dot = x_nearest2;

        double dx1 = x_rand1 - x_nearest1;
        // xdot = dx/dt so dt = dx/xdot
        double dt = dx1 / x1dot;
        if (timeForward) {
            if (dt <= MIN_DT)
                return null;
            if (dt > MAX_DT)
                dt = MAX_DT;
        } else {
            if (dt >= -1.0*MIN_DT)
                return null;
            if (dt < -1.0*MAX_DT)
                dt = -1.0*MAX_DT;
        }

        // from system dynamics
        // x2dot = - g * sin(x1) / l + u
        // u = dx2/dt + g*sin(x1)

        double dx2 = x_rand2 - x_nearest2;
        // TODO: sign of u means ???
        double u;
        // if (timeForward) {
            u = dx2 / dt + _g * Math.sin(x_nearest1) / l;
        // } else {
            // System.out.println("time reversed");
            // u = dx2 / dt - _g * Math.sin(x_rand1) / l;
        // }
        // just skip x_rand that are too hard
        // System.out.printf("u %5.3f\n", u);
        // if (u < -1 * MAX_U) return null;
        // if (u > MAX_U) return null;

        // // for now this is really just an effort filter.
        // return new double[] { x_rand1, x_rand2 };

        u = Math.max(MIN_U, u);
        u = Math.min(MAX_U, u);

        double x2dot;
        // if (timeForward) {
            x2dot = -1 * _g * Math.sin(x_nearest1) / l + u;
        // } else {
            // x2dot = _g * Math.sin(x_rand1) / l + u;
        // }

        double x_new1 = x_nearest1 + x1dot * dt + 0.5 * x2dot * dt * dt;
        double x_new2 = x_nearest2 + x2dot * dt;

        System.out.printf("from [%5.3f %5.3f] to [%5.3f %5.3f] xdot [%5.3f %5.3f] dt %5.3f u %5.3f\n",
                x_nearest1, x_nearest2, x_new1, x_new2, x1dot, x2dot, dt, u);
        return new double[] { x_new1, x_new2 };

        // see pend_rrt.m
        // Matrix<N2, N1> x_near = VecBuilder.fill(nearConfig[0], nearConfig[1]);
        // Matrix<N2, N1> x_rand = VecBuilder.fill(newConfig[0], newConfig[1]);
        // Matrix<N2, N1> dx = x_near.minus(x_rand);
        // Matrix<N1, N2> K = getK(newConfig);
        // double u = K.times(-1).times(dx).get(0, 0);
        // u = Math.max(-3, u);
        // u = Math.min(3, u);
        // Matrix<N2, N1> xdot = VecBuilder.fill(
        // nearConfig[1],
        // (u - b * nearConfig[1] - m * _g * l * Math.sin(nearConfig[0])));
        // Matrix<N2, N1> x_new = x_near.plus(xdot.times(h));
        // return x_new.getData();
        // // newConfig[0] = x_new.get(0, 0);
        // // newConfig[1] = x_new.get(1, 0);
        // return new double[] { 1, 1 };
    }

    @Override
    public double[] initial() {
        return _init;
    }

    @Override
    public double[] goal() {
        return _goal;
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
