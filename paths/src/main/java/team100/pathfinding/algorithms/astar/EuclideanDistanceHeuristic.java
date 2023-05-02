package team100.pathfinding.algorithms.astar;

import team100.geometry.Point;

public class EuclideanDistanceHeuristic{
    private final Point target;

    public EuclideanDistanceHeuristic(Point target) {
        this.target = target;
    }

    public double estimate(Point point) {
        double x = Math.pow(target.getX() - point.getX(), 2.0);
        double y = Math.pow(target.getY() - point.getY(), 2.0);
        
        return Math.sqrt(x + y);
    }
}