package team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;

import team100.geometry.Point;
import team100.geometry.Polygon;
import team100.graph.Connection;
import team100.visibilitygraph.VisibilityGraph;

public class VisibilityGraphTest {
    private static final double kEpsilon = 0.1;

    @Test
    public void testOne() {
        Polygon p = new Polygon();
        Point start = new Point(0, 0);
        Point end = new Point(100, 100);
        VisibilityGraph v = new VisibilityGraph(start, end, p);
        List<Connection<Point>> l = v.getConnections(start);
        assertEquals(1, l.size());

        Connection<Point> c = l.get(0);
        assertEquals(141.4, c.getCost(), kEpsilon);

        Point from = c.getFrom();
        assertEquals(0, from.getX());
        assertEquals(0, from.getY());

        Point to = c.getTo();
        assertEquals(100, to.getX());
        assertEquals(100, to.getY());
    }

}
