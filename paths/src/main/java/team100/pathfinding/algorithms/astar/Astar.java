package team100.pathfinding.algorithms.astar;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import team100.geometry.Point;
import team100.graph.Connection;
import team100.graph.Path;
import team100.pathfinding.algorithms.PathFindingList;
import team100.visibilitygraph.VisibilityGraph;

public class Astar {
    private final EuclideanDistanceHeuristic heuristic;
    private final VisibilityGraph graph;
    private final PathFindingList<Point> openList;
    private final PathFindingList<Point> closedList;

    public Astar(EuclideanDistanceHeuristic heuristic, VisibilityGraph graph) {
        this.heuristic = heuristic;
        this.graph = graph;
        openList = new PathFindingList<>();
        closedList = new PathFindingList<>();

        AstarNode<Point> startRecord = new AstarNode<>(graph.getStart(), null, 0, heuristic.estimate(graph.getStart()));
        openList.insert(startRecord);
    }

    public Path<Point> find() {
        AstarNode<Point> current = null;
        while (!openList.isEmpty()) {
            current = (AstarNode<Point>) openList.removeSmallest();
            if (current.getNode().equals(graph.getEnd()))
                break;

            List<Connection<Point>> outgoings = graph.getConnections(current.getNode());

            for (Connection<Point> connection : outgoings) {
                Point endNode = connection.getTo();
                double endNodeCost = current.getCostSoFar() + connection.getCost();
                double endNodeHeuristic;
                if (closedList.contains(endNode)) {
                    AstarNode<Point> endRecord = (AstarNode<Point>) closedList.find(endNode);
                    // If it is not a better path just ignore
                    if (endRecord.getCostSoFar() <= endNodeCost)
                        continue;

                    closedList.remove(endNode);
                    endNodeHeuristic = endRecord.getEstimatedTotalCost() - endRecord.getCostSoFar();

                } else if (openList.contains(endNode)) {
                    AstarNode<Point> endRecord = (AstarNode<Point>) openList.find(endNode);
                    if (endRecord.getCostSoFar() <= endNodeCost)
                        continue;

                    endNodeHeuristic = endRecord.getEstimatedTotalCost() - endRecord.getCostSoFar();
                } else {
                    endNodeHeuristic = heuristic.estimate(endNode);
                }

                AstarNode<Point> endNodeRecord = new AstarNode<>(endNode, connection, endNodeCost, endNodeCost + endNodeHeuristic);

                if (!openList.contains(endNode))
                    openList.insert(endNodeRecord);
                else openList.update(endNodeRecord);

            }

            closedList.insert(current);
        }

        if (current == null || !current.getNode().equals(graph.getEnd()))
            return null;

        return buildPath(current, closedList);
    }

    private Path<Point> buildPath(AstarNode<Point> current, PathFindingList<Point> closed) {
        List<Connection<Point>> connections = new ArrayList<>();

        while (!current.getNode().equals(graph.getStart())) {
            Connection<Point> connection = current.getConnection();
            connections.add(connection);
            current = closed.find(connection.getFrom());
        }

        Collections.reverse(connections);
        Path<Point> path = new Path<>();
        path.setEdges(connections);
        return path;
    }
}