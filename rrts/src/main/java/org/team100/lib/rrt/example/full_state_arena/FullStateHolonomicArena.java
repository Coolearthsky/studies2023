package org.team100.lib.rrt.example.full_state_arena;

import java.awt.Color;

import org.team100.lib.example.Arena;
import org.team100.lib.geom.Obstacle;
import org.team100.lib.geom.Polygon;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNearNode;

/**
 * this only works for 4d full state
 * TODO: remove the whole "dimensions" idea and all these arrays, use type-safe
 * wpi stuff
 */
public class FullStateHolonomicArena implements Arena {
    private static final double DISCRETIZATION = 0.25;
    private static final double ROBOT_RADIUS = .4;
    private static final double GOAL_RADIUS = 0.4;
    private static final int DIMENSIONS = 4;

    // init and goal are motionless
    private static final double[] _init = { 15.5, 0, 6.75, 0 };
    private static final double[] _goal = { 1.93, 0, 2.748, 0 };
    private static final double[] _min = { 0, -6, 0, -6 };
    private static final double[] _max = { 16, 6, 8, 6 };

    // used for steering
    // private final double _gamma;
    // private int stepNo;
    private double radius;

    Obstacle[] _obstacles = new Obstacle[] {
            // see studies2023/glc
            // nodes
            new Polygon(Color.RED, 0, 0, 1.43, 0, 1.43, 5.49, 0, 5.49),
            // community
            new Polygon(Color.BLUE, 13.18, 0, 16, 0, 16, 5.49, 13.18, 5.49),
            // opponents
            new Polygon(Color.BLUE, 8, 4, 9, 4, 9, 5, 8, 5),
            new Polygon(Color.BLUE, 10, 5, 11, 5, 11, 6, 10, 6),
            new Polygon(Color.BLUE, 9, 6, 10, 6, 10, 7, 9, 7),
            // alliance-mate
            new Polygon(Color.RED, 6, 5, 7, 5, 7, 6, 6, 6),
            new Polygon(Color.RED, 4, 5, 5, 5, 5, 6, 4, 6),

            // loading
            new Polygon(Color.BLUE, 0, 8, 3.36, 8, 3.36, 5.49, 0, 5.49),
            // charge stations
            new Polygon(Color.RED, 2.98, 1.51, 4.91, 1.51, 4.91, 3.98, 2.98, 3.98),
            new Polygon(Color.BLUE, 11.63, 1.51, 13.56, 1.51, 13.56, 3.98, 11.63, 3.98)
    };

    public FullStateHolonomicArena() {
        // _gamma = gamma;
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

    @Override
    public double dist(double[] start, double[] end) {
        double dist = 0;
        for (int i = 0; i < DIMENSIONS; i += 2) {
            double dx = start[i] - end[i];
            double dy = start[i + 1] - end[i + 1];
            dist += dx * dx + dy * dy;
        }
        return Math.sqrt(dist);
    }

    @Override
    public void setStepNo(int stepNo) {
        // this.stepNo = stepNo;
    }

    @Override
    public void setRadius(double radius) {
        this.radius = radius;
    }

    @Override
    public double[] steer(KDNearNode<Node> x_nearest, double[] x_rand) {
        double dist = radius / x_nearest._dist;

        // if it's close enough then just return it
        if (x_nearest._dist < radius)
            return x_rand;

        double[] nearConfig = x_nearest._nearest.getState();
        double[] result = new double[DIMENSIONS];
        for (int i = 0; i < DIMENSIONS; ++i) {
            result[i] = nearConfig[i] + (x_rand[i] - nearConfig[i]) * dist;
        }
        return result;

    }

    /**
     * config is (x xdot y ydot)
     * so we check x and y only
     */
    @Override
    public boolean clear(double[] config) {
        // robot-obstacle collision
        for (Obstacle obstacle : _obstacles) {
            if (obstacle.distToPoint(config[0], config[2]) < ROBOT_RADIUS)
                return false;
        }
        return true;
    }

    @Override
    public boolean link(double[] a, double[] b) {
        double[] dx = new double[DIMENSIONS];
        double dist = 0;
        for (int i = 0; i < DIMENSIONS; ++i) {
            dx[i] = b[i] - a[i];
            dist += dx[i] * dx[i];
        }

        dist = Math.sqrt(dist);

        int steps = (int) Math.floor(dist / DISCRETIZATION) + 2;

        double[] p = new double[DIMENSIONS];

        for (int i = 0; i <= steps; ++i) {
            for (int j = 0; j < DIMENSIONS; ++j) {
                p[j] = (a[j] * (steps - i) + b[j] * i) / steps;
            }
            if (!clear(p)) {
                return false;
            }
        }

        return true;
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
    public boolean goal(double[] conf) {
        return dist(conf, _goal) < GOAL_RADIUS;
    }

    public Obstacle[] obstacles() {
        return _obstacles;
    }
}
