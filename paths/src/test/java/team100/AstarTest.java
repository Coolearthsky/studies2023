package team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import team100.geometry.Point;
import team100.geometry.Polygon;
import team100.graph.Connection;
import team100.graph.Path;
import team100.pathfinding.algorithms.astar.Astar;
import team100.pathfinding.algorithms.astar.EuclideanDistanceHeuristic;
import team100.visibilitygraph.VisibilityGraph;

public class AstarTest {
    private static final double kEpsilon = 0.1;

    @Test
    public void testNoObstacle() {
        // just goes straight from corner to corner
        Polygon p = new Polygon();
        Point start = new Point(0, 0);
        Point end = new Point(100, 100);
        VisibilityGraph v = new VisibilityGraph(start, end, p);

        Astar astar = new Astar(new EuclideanDistanceHeuristic(end), v);
        Path<Point> path = astar.find();
        assertEquals(1, path.size());

        Connection<Point> c = path.get(0);
        assertEquals(141.4, c.getCost(), kEpsilon);

        Point from = c.getFrom();
        assertEquals(0, from.getX());
        assertEquals(0, from.getY());

        Point to = c.getTo();
        assertEquals(100, to.getX());
        assertEquals(100, to.getY());
    }

    @Test
    public void testOneObstacle() {
        // make an obstacle to avoid; the path has two segments
        // to avoid the obstacle.
        Polygon p = new Polygon(
                new Point(45, 45),
                new Point(45, 55),
                new Point(55, 55),
                new Point(55, 45));
        Point start = new Point(0, 0);
        Point end = new Point(100, 100);
        VisibilityGraph v = new VisibilityGraph(start, end, p);

        Astar astar = new Astar(new EuclideanDistanceHeuristic(end), v);
        Path<Point> path = astar.find();
        assertEquals(2, path.size());

        Connection<Point> c0 = path.get(0);
        assertEquals(71.1, c0.getCost(), kEpsilon);

        Point from0 = c0.getFrom();
        assertEquals(0, from0.getX());
        assertEquals(0, from0.getY());

        Point to0 = c0.getTo();
        assertEquals(45, to0.getX());
        assertEquals(55, to0.getY());

        Connection<Point> c1 = path.get(1);
        assertEquals(71.1, c1.getCost(), kEpsilon);

        Point from1 = c1.getFrom();
        assertEquals(45, from1.getX());
        assertEquals(55, from1.getY());

        Point to1 = c1.getTo();
        assertEquals(100, to1.getX());
        assertEquals(100, to1.getY());
    }

}
