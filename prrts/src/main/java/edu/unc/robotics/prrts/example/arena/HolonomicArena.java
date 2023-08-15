package edu.unc.robotics.prrts.example.arena;

import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;

import edu.unc.robotics.prrts.RobotModel;
import edu.unc.robotics.prrts.example.geom.Circle;
import edu.unc.robotics.prrts.example.geom.Obstacle;
import edu.unc.robotics.prrts.example.geom.Polygon;
import edu.unc.robotics.prrts.kdtree.KDModel;

/**
 * HolonomicArena2D
 *
 * @author jeffi
 */
public class HolonomicArena implements RobotModel, KDModel {
    private static final double DISCRETIZATION = 0.25;
    private static final double ROBOT_RADIUS = .4;
    private static final double GOAL_RADIUS = 0.4;
    private final static int DIMENSIONS = 2;

    double _eta = 1.0;

    double[] _goal = { 2.7, 9.0, 1, 1, 1, 9, 9, 1 };
    Shape _goalShape;
    double[] _min = { 0, 0, 0, 0, 0, 0, 0, 0 };
    double[] _max = { 10, 10, 10, 10, 10, 10, 10, 10 };

    Obstacle[] _obstacles = new Obstacle[] {
            new Circle(2.0, 2.5, 1.1), // dinner table
            new Polygon(0.5, 4.5, 2.1, 4.5, 2.1, 8.0, 0.5, 8.0), // couch
            new Circle(1.5, 9.0, 1.5 / 2), // end table
            new Polygon(3.9, 8.5, 4.2, 9.9, 6.2, 9.5, 5.9, 8.1), // blue chair
            new Polygon(4.1, 5.4, 5.5, 5.4, 5.5, 7.0, 4.1, 7.0), // coffee table
            new Polygon(4.5, 2.5, 7.1, 2.5, 7.1, 3.5, 4.5, 3.5), // island
            new Circle(3.6, 4.3, 0.35), // chair v2
            new Polygon(9.2, 2.5, 10.0, 2.5, 10.0, 3.5, 9.2, 3.5), // fridge
            new Circle(6.6, 4.1, 0.5), // toy
            new Polygon(9.2, 4.5, 10.0, 4.5, 10.0, 9.0, 9.2, 9.0), // wall
            new Circle(2.4, 7.0, 0.3), // ottoman
    };

    public HolonomicArena() {


        Area goalShape = new Area();

        goalShape.add(new Area(new Ellipse2D.Double(
                _goal[0] - GOAL_RADIUS, _goal[1] - GOAL_RADIUS, GOAL_RADIUS, GOAL_RADIUS)));

        _goalShape = goalShape;
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

    @Override
    public double dist(double[] a, double[] b) {
        double dist = 0;
        for (int i = 0; i < DIMENSIONS; i += 2) {
            double dx = a[i] - b[i];
            double dy = a[i + 1] - b[i + 1];
            dist += dx * dx + dy * dy;
        }
        return Math.sqrt(dist);
    }

    public double steer(double[] a, double[] b, double dist) {
        if (dist < _eta) {
            return dist;
        } else {
            double scale = _eta / dist;
            for (int i = 0; i < DIMENSIONS; ++i) {
                b[i] = a[i] + (b[i] - a[i]) * scale;
            }
            return _eta;
        }
    }

    @Override
    public boolean clear(double[] config) {
        // robot-obstacle collision
        for (Obstacle obstacle : _obstacles) {
            for (int j = 0; j < DIMENSIONS; j += 2) {
                if (obstacle.distToPoint(config[j], config[j + 1]) < ROBOT_RADIUS) {
                    return false;
                }
            }
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
    public boolean goal(double[] conf) {
        return dist(conf, _goal) < GOAL_RADIUS;
    }

    public Obstacle[] obstacles() {
        return _obstacles;
    }
}
