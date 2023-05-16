package team100.pathfinding.algorithms.astar;

import team100.graph.Connection;

public class AstarNode<Node> {
    private final Node node;
    private final Connection<Node> connection;
    private final double costSoFar;
    private final double estimatedTotalCost;

    public AstarNode(Node node, Connection<Node> connection, double costSoFar, double estimatedTotalCost) {
        this.node = node;
        this.connection = connection;
        this.costSoFar = costSoFar;
        this.estimatedTotalCost = estimatedTotalCost;
    }

    public double getEstimatedTotalCost() {
        return estimatedTotalCost;
    }

    public Node getNode() {
        return node;
    }

    public double getCostSoFar() {
        return costSoFar;
    }

    public double getCost() {
        return estimatedTotalCost;
    }

    public Connection<Node> getConnection() {
        return connection;
    }
}