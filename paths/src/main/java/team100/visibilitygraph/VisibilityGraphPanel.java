package team100.visibilitygraph;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;

import team100.geometry.Point;
import team100.geometry.Polygon;
import team100.graph.Connection;
import team100.graph.Path;
import team100.pathfinding.algorithms.astar.Astar;
import team100.pathfinding.algorithms.astar.EuclideanDistanceHeuristic;

public class VisibilityGraphPanel extends JPanel {
    private static final int kWindowHeight = 800;
    private static final int kWindowWidth = 800;
    private static final float kShortestPathLineWidth = 4.0F;
    private static final Color kLineColor = Color.black;
    private static final Color kFullCellColor = Color.blue;

    private static final float kOffsetX = -10;
    private static final float kCellWidth = 20;
    private static final float kOffsetY = -10;
    private static final float kCellHeight = 20;

    public VisibilityGraph visibilityGraph;
    private Path<Point> path;

    private final JFrame frame;

    public VisibilityGraphPanel(VisibilityGraph visibilityGraph) {
        this.visibilityGraph = visibilityGraph;
        frame = new JFrame(String.format("Visibilty Graph"));
        frame.add(this);
        frame.setSize(kWindowWidth + 30, kWindowHeight + 60);
        frame.setLocation((kWindowWidth + 30), 0);
        frame.setVisible(true);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }

    public void prepareForRender() {
        Astar astar = new Astar(new EuclideanDistanceHeuristic(visibilityGraph.getEnd()),visibilityGraph);
        path = astar.find();
    }

    private void renderPolygon(Graphics2D g, Polygon polygon, Color fillColor) {
        g.setColor(fillColor);

        java.awt.Polygon p = new java.awt.Polygon();
        for (int i = 0; i < polygon.getPointCount(); ++i) {
            Point point = polygon.getPoint(i);
            p.addPoint((int) point.getX(), (int) point.getY());
        }
        g.fill(p);
        g.draw(p);
    }

    private void renderLine(Graphics2D g, Connection<Point> connection, Color fillColor) {
        g.setColor(fillColor);

        g.drawLine((int) connection.getFrom().getX(),
                (int) connection.getFrom().getY(),
                (int) connection.getTo().getX(),
                (int) connection.getTo().getY());
    }

    private void renderStartAndEnd(Graphics2D g, Point startPoint, Point endPoint, Color fillColor) {
        g.setColor(fillColor);
        renderPoint(g, startPoint, fillColor);
        renderPoint(g, endPoint, fillColor);
    }

    private void renderPoint(Graphics2D g, Point node, Color fillColor) {
        float x = kOffsetX + node.getX();
        float y = kOffsetY + node.getY();

        g.setColor(fillColor);
        g.fillOval((int) x, (int) y, (int) kCellWidth, (int) kCellHeight);

        g.setColor(kLineColor);
        g.drawOval((int) x, (int) y, (int) kCellWidth, (int) kCellHeight);
    }

    private void renderPath(Graphics2D g) {
        if (path == null) return;

        if (path.isEmpty())
            return;

        g.setStroke(new BasicStroke(kShortestPathLineWidth));

        for (int i = 0; i < path.size(); i++) {
            renderLine(g, path.get(i), Color.green);
        }
    }

    public synchronized void paintComponent(Graphics graphics) {
        super.paintComponent(graphics);
        Graphics2D g = (Graphics2D) graphics;

        for (Polygon p: visibilityGraph.getPolygons()) {
            renderPolygon(g, p, kFullCellColor);
        }

        renderStartAndEnd(g, visibilityGraph.getStart(), visibilityGraph.getEnd(), Color.red);

        for (Point currentPoint: visibilityGraph.getPoints()) {
            List<Connection<Point>> currentConnections = visibilityGraph.getConnections(currentPoint);
            if (currentConnections == null)
                continue;
            for (Connection<Point> currentConnection : currentConnections) {
                renderLine(g, currentConnection, Color.red);
            }
        }

        renderPath(g);
    }

}
