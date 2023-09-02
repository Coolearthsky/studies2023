package team100.visibilitygraph;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import team100.geometry.Line;
import team100.geometry.Point;
import team100.geometry.Polygon;
import team100.graph.Connection;
import team100.pathfinding.algorithms.astar.EuclideanDistanceHeuristic;

public class VisibilityGraph {
    private final Point startPoint;
    private final Point endPoint;
    private final List<Polygon> polygons;
    private final List<Point> nodes;
    private final HashMap<Point, List<Connection<Point>>> connections;

    public VisibilityGraph(Point start, Point end, Polygon... polygons) {
        startPoint = start;
        endPoint = end;
        this.polygons = new ArrayList<Polygon>(Arrays.asList(polygons));
        nodes = new ArrayList<Point>();
        connections = new HashMap<Point, List<Connection<Point>>>();

        nodes.add(start);
        nodes.add(end);

        for (Polygon polygon : this.polygons) {
            nodes.addAll(polygon.getPoints());
        }

        for (int i = 0; i < nodes.size(); i++) {
            List<Connection<Point>> connectionList = new ArrayList<>();
        
            for (int j = 0; j < nodes.size(); j++) {
                if (j != i && !intersects(nodes.get(i), nodes.get(j))) {
                    EuclideanDistanceHeuristic heuristic = new EuclideanDistanceHeuristic(nodes.get(i));
                    connectionList.add(new Connection<>(nodes.get(i),
                            nodes.get(j),
                            heuristic.estimate(nodes.get(j))));
                }
            }
        
            connections.put(nodes.get(i), connectionList);
        }
    }

    public Point getStart() {
        return startPoint;
    }
    
    public Point getEnd() {
        return endPoint;
    }

    public List<Point> getPoints() {
        return Collections.unmodifiableList(nodes);
    }

    public List<Polygon> getPolygons() {
        return Collections.unmodifiableList(polygons);
    }

    private boolean intersects(Point point1, Point point2) {
        for (Polygon polygon : polygons) {
            float x1 = point1.getX();
            float y1 = point1.getY();
            float x2 = point2.getX();
            float y2 = point2.getY();
            if (new Line(point1, point2).intersects(polygon)
                    && new Line(new Point(x1 + 1, y1), point2).intersects(polygon)
                    && new Line(new Point(x1 - 1, y1), point2).intersects(polygon)
                    && new Line(new Point(x1, y1 + 1), point2).intersects(polygon)
                    && new Line(new Point(x1 + 1, y1 + 1), new Point(x2 + 1, y2 + 1)).intersects(polygon)
                    && new Line(new Point(x1, y1 - 1), point2).intersects(polygon)
                    && new Line(new Point(x1 - 1, y1 - 1), point2).intersects(polygon)
                    && new Line(new Point(x1 + 1, y1 - 1), point2).intersects(polygon)
                    && new Line(new Point(x1 - 1, y1 + 1), point2).intersects(polygon)
                    && new Line(point1, new Point(x2 + 1, y2)).intersects(polygon)
                    && new Line(point1, new Point(x2 - 1, y2)).intersects(polygon)
                    && new Line(point1, new Point(x2, y2 + 1)).intersects(polygon)
                    && new Line(point1, new Point(x2, y2 - 1)).intersects(polygon)
                    && new Line(new Point(x1, y1 - 1), new Point(x2, y2 - 1)).intersects(polygon)
                    && new Line(point1, new Point(x2 + 1, y2 - 1)).intersects(polygon)
                    && new Line(point1, new Point(x2 - 1, y2 + 1)).intersects(polygon)) {
                return true;
            }
        }

        return false;
    }

    public List<Connection<Point>> getConnections(Point from) {
        return connections.get(from);
    }
}