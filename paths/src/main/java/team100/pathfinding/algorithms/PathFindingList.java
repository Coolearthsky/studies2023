package team100.pathfinding.algorithms;

import java.util.HashMap;

import org.jheaps.AddressableHeap;
import org.jheaps.tree.FibonacciHeap;

import team100.pathfinding.algorithms.astar.AstarNode;

public class PathFindingList<Node> {
    private final FibonacciHeap<Double, Node> heap;
    private final HashMap<Node, AddressableHeap.Handle<Double, Node>> fibHash;
    private final HashMap<Node, AstarNode<Node>> pathFindingHash;

    public PathFindingList() {
        heap = new FibonacciHeap<>();
        fibHash = new HashMap<>();
        pathFindingHash = new HashMap<>();
    }

    public void insert(AstarNode<Node> record) {
        AddressableHeap.Handle<Double, Node> handle = heap.insert(record.getCost(), record.getNode());
        fibHash.put(record.getNode(), handle);
        pathFindingHash.put(record.getNode(), record);
    }

    public boolean isEmpty() {
        return heap.isEmpty();
    }

    public AstarNode<Node> removeSmallest() {
        Node min = heap.findMin().getValue();
        AstarNode<Node> minRecord = pathFindingHash.get(min);
        fibHash.remove(min);
        pathFindingHash.remove(min);
        heap.deleteMin();
        return minRecord;
    }

    public boolean contains(Node node) {
        return fibHash.containsKey(node);
    }

    public AstarNode<Node> find(Node node) {
        return pathFindingHash.get(node);
    }

    public void update(AstarNode<Node> record) {
        if (!contains(record.getNode()))
            return;
        fibHash.get(record.getNode()).decreaseKey(record.getCost());
        pathFindingHash.put(record.getNode(), record);
    }

    public void remove(Node endNode) {
        if (!fibHash.containsKey(endNode))
            return;
        fibHash.remove(endNode).delete(); // remove from the hash and delete from the heap
        pathFindingHash.remove(endNode);
    }
}