package org.team100.lib.rrt.example.full_state_arena;

import java.awt.Color;

import org.team100.lib.example.Arena;
import org.team100.lib.geom.Obstacle;
import org.team100.lib.geom.Polygon;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNearNode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

/**
 * this only works for 4d full state
 */
public class FullStateHolonomicArena implements Arena<N4> {
    private static final double DISCRETIZATION = 0.25;
    private static final double ROBOT_RADIUS = .4;
    private static final double GOAL_RADIUS = 0.4;

    // init and goal are motionless

    private static final Matrix<N4, N1> _init = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 15.5, 0, 6.75, 0 });
    private static final Matrix<N4, N1> _goal = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1.93, 0, 2.748, 0 });
    // private static final Matrix<N4, N1> _init = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 10, 0, 4, 0 });
    // private static final Matrix<N4, N1> _goal = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 6, 0, 2, 0 });
   
    private static final Matrix<N4, N1> _min = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 0, -3, 0, -3 });
    private static final Matrix<N4, N1> _max = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 16, 3, 8, 3 });

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
    public Matrix<N4, N1> getMin() {
        return _min.copy();
    }

    @Override
    public Matrix<N4, N1> getMax() {
        return _max.copy();
    }

    @Override
    public double dist(Matrix<N4, N1> start, Matrix<N4, N1> end) {
        return start.minus(end).normF();
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
    public Matrix<N4, N1> steer(KDNearNode<Node<N4>> x_nearest, Matrix<N4, N1> x_rand) {
        double dist = radius / x_nearest._dist;
        // if it's close enough then just return it
        if (x_nearest._dist < radius)
            return x_rand;
        Matrix<N4, N1> nearConfig = x_nearest._nearest.getState();
        return nearConfig.plus(x_rand.minus(nearConfig).times(dist));
    }

    /**
     * config is (x xdot y ydot)
     * so we check x and y only
     */
    @Override
    public boolean clear(Matrix<N4, N1> config) {
        if (config.get(0, 0) - ROBOT_RADIUS < _min.get(0, 0))
            return false;
        if (config.get(2, 0) - ROBOT_RADIUS < _min.get(2, 0))
            return false;
        if (config.get(0, 0) + ROBOT_RADIUS > _max.get(0, 0))
            return false;
        if (config.get(2, 0) + ROBOT_RADIUS > _max.get(2, 0))
            return false;
            // poor-man's velocity limit.  TODO: add bang-cruise-bang solutions.
        if (config.get(1, 0) < _min.get(1, 0))
            return false;
        if (config.get(3, 0) < _min.get(3, 0))
            return false;
        if (config.get(1, 0) > _max.get(1, 0))
            return false;
        if (config.get(3, 0) > _max.get(3, 0))
            return false;

        // robot-obstacle collision
        for (Obstacle obstacle : _obstacles) {
            if (obstacle.distToPoint(config.get(0, 0), config.get(2, 0)) < ROBOT_RADIUS)
                return false;
        }
        return true;
    }

    @Override
    public boolean link(Matrix<N4, N1> a, Matrix<N4, N1> b) {
        double dist = b.minus(a).normF();
        int steps = (int) Math.floor(dist / DISCRETIZATION) + 2;
        for (int i = 0; i <= steps; ++i) {
            Matrix<N4, N1> p = a.times(steps - i).plus(b.times(i)).div(steps);
            if (!clear(p)) {
                return false;
            }
        }
        return true;
    }

    @Override
    public Matrix<N4, N1> initial() {
        return _init;
    }

    @Override
    public Matrix<N4, N1> goal() {
        return _goal;
    }

    @Override
    public boolean goal(Matrix<N4, N1> conf) {
        return dist(conf, _goal) < GOAL_RADIUS;
    }

    public Obstacle[] obstacles() {
        return _obstacles;
    }
}
