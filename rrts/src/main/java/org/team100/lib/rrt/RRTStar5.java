package org.team100.lib.rrt;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;

import org.team100.lib.graph.Graph;
import org.team100.lib.graph.LinkInterface;
import org.team100.lib.graph.NearNode;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDModel;
import org.team100.lib.index.KDNearNode;
import org.team100.lib.index.KDNode;
import org.team100.lib.index.KDTree;
import org.team100.lib.planner.RobotModel;
import org.team100.lib.planner.Solver;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;

/**
 * RRT* version 5. this is the bang-bang full-state version
 * 
 * This is an attempt to make the code below look more like the pseudocode.
 * 
 * There are many similar-but-not-identical variations; I've kind of mashed them
 * together.
 * 
 * x_rand <- Sample() // a random point
 * x_nearest <- Nearest(x_rand) // the nearest node in the tree
 * x_new <- Steer(x_nearest, x_rand) // a feasible point nearby
 * if CollisionFree then
 * | X_near = Near(x_new) // candidate parents
 * | x_min = ChooseParent(X_near, x_nearest, x_new) // lowest cost parent
 * | if x_min != null then
 * | | newNode = InsertNode(x_min, x_new)
 * | | Rewire(X_near, newNode);
 * 
 * References:
 * 
 * http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf
 * https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/SamplingBasedMotionPlanning.pdf
 * http://lavalle.pl/planning/book.pdf
 * https://dspace.mit.edu/handle/1721.1/78449
 * https://dspace.mit.edu/handle/1721.1/79884
 * https://natanaso.github.io/ece276b2018/ref/ECE276B_8_SamplingBasedPlanning.pdf
 * https://dspace.mit.edu/bitstream/handle/1721.1/79884/MIT-CSAIL-TR-2013-021.pdf
 * 
 * This version implements bidirectional search, which is consisely mentioned
 * here
 * 
 * https://arxiv.org/pdf/1703.08944.pdf
 */
public class RRTStar5<T extends KDModel & RobotModel> implements Solver {
    private final T _model;
    /** Initially, tree grown from initial, but is swapped repeatedly */
    private KDNode<Node> _T_a;
    /** Initially, tree grown from goal, but is swapped repeatedly */
    private KDNode<Node> _T_b;
    private final Sample _sample;
    private final double _gamma;
    /** Lowest cost leaf leading to initial state. */
    // TODO: remove this
    private LinkInterface _bestLeaf_a;

    // mutable loop variables to make the loop code cleaner
    int stepNo;
    double radius;

    Path _sigma_best;
    Map<Node, Node> connections = new HashMap<Node, Node>();

    public RRTStar5(T model, Sample sample, double gamma) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        _model = model;
        _T_a = new KDNode<Node>(new Node(model.initial()));
        _T_b = new KDNode<Node>(new Node(model.goal()));
        _sample = sample;
        _gamma = gamma;
        _bestLeaf_a = null;
    }

    /**
     * Note this isn't quite the same as https://arxiv.org/pdf/1703.08944.pdf
     * because it doesn't use Extend, so it doesn't try to connect unless
     * a new node is actually inserted.
     * 
     * @return true if a new sample was added.
     */
    @Override
    public int step() {
        int edges = 0;
        double[] x_rand = SampleFree();
        KDNearNode<Node> x_nearest = Nearest(x_rand, _T_a);
        double[] x_new = Steer(x_nearest, x_rand);
        if (x_new == null) return 0;
        if (CollisionFree(x_nearest._nearest.getState(), x_new)) {
            List<NearNode> X_near = Near(x_new, _T_a);
            if (X_near.isEmpty())
                X_near.add(new NearNode(x_nearest._nearest, x_nearest._dist));
            Node x_min = ChooseParent(X_near, x_new);
            if (x_min != null) {
                Node newNode = InsertNode(x_min, x_new, _T_a);
                Rewire(X_near, newNode);
                edges += 1;
                KDNearNode<Node> x_conn = Nearest(x_new, _T_b);
                Path sigma_new = Connect(newNode, x_conn, _T_b);
                if (sigma_new != null) {
                    edges += 1;
                    if (_sigma_best == null) {
                        _sigma_best = sigma_new;
                    } else {
                        if (sigma_new.getDistance() < _sigma_best.getDistance()) {
                            _sigma_best = sigma_new;
                        }
                    }
                }
            }
        }
        SwapTrees();
        return edges;
    }

    void SwapTrees() {
        KDNode<Node> tmp = _T_a;
        _T_a = _T_b;
        _T_b = tmp;
    }

    /**
     * this isn't quite the same as https://arxiv.org/pdf/1703.08944.pdf
     * because it skips the "extend," because i think it's wrong; it just
     * picks nodes in the other tree that are actually near the new node.
     * 
     * @param x_1 newly inserted node
     * @param x_2 near node in the other tree
     */
    Path Connect(Node x_1, KDNearNode<Node> x_2, KDNode<Node> rootNode) {
        if (CollisionFree(x_2._nearest.getState(), x_1.getState())) {
            List<NearNode> X_near = Near(x_1.getState(), rootNode);
            if (X_near.isEmpty())
                X_near.add(new NearNode(x_2._nearest, x_2._dist));
            Node x_min = ChooseParent(X_near, x_1.getState());
            if (x_min != null) {
                Node newNode = InsertNode(x_min, x_1.getState(), rootNode);
                connections.put(x_1, newNode);
                Rewire(X_near, newNode);
                return GeneratePath(x_1, newNode);
            }
        }
        return null;
    }

    /**
     * the parameters describe a link between initial and goal trees, the same
     * state in both cases.
     */
    Path GeneratePath(Node x_1, Node x_2) {
        for (int i = 0; i < _model.dimensions(); ++i) {
            if (x_1.getState()[i] != x_2.getState()[i])
                throw new IllegalArgumentException(
                        "x1 " + Arrays.toString(x_1.getState()) + " x2 " + Arrays.toString(x_2.getState()));
        }

        Path p_1 = walkParents(x_1);
        Path p_2 = walkParents(x_2);
        List<double[]> states_2 = p_2.getStates();
        Collections.reverse(states_2);
        List<double[]> fullStates = new ArrayList<double[]>();
        fullStates.addAll(p_1.getStates());
        fullStates.addAll(states_2);
        return new Path(p_1.getDistance() + p_2.getDistance(), fullStates);
    }

    boolean CollisionFree(double[] from, double[] to) {
        return _model.link(from, to);
    }

    /** Return a state not within an obstacle. */
    double[] SampleFree() {
        while (true) {
            double[] newConfig = _sample.get();
            if (_model.clear(newConfig))
                return newConfig;
        }
    }

    /**
     * Return the nearest node in the tree, using the KDTree metric, which
     * could be very wrong but is probably useful.
     * 
     * @param x_rand   the random sample
     * @param rootNode the tree to look through
     */
    KDNearNode<Node> Nearest(double[] x_rand, KDNode<Node> rootNode) {
        return KDTree.nearest(_model, rootNode, x_rand);
    }

    /**
     * Return a state that tries to go from x_nearest to x_rand using a feasible
     * trajectory.
     * 
     * @param x_nearest starting state
     * @param x_rand    goal state
     * @return x_new a feasible state
     */
    double[] Steer(KDNearNode<Node> x_nearest, double[] x_rand) {
        _model.setStepNo(stepNo);
        _model.setRadius(radius);
        return _model.steer(x_nearest, x_rand);
    }

    /**
     * Return a list of nearby nodes, using the KDTree metric, which may not
     * actually contain the nearest nodes in non-Euclidean spaces. Returns the
     * single
     * nearest node if there are no other near nodes.
     */
    List<NearNode> Near(double[] x_new, KDNode<Node> rootNode) {
        List<NearNode> nearNodes = new ArrayList<>();
        KDTree.near(_model, rootNode, x_new, radius, (node, dist) -> {
            nearNodes.add(new NearNode(node, dist));
        });
        return nearNodes;
    }

    /**
     * Returns a member of X_near resulting in the lowest-cost path to x_new.
     * Removes infeasible nodes from X_near so we don't look at them again later.
     */
    Node ChooseParent(List<NearNode> X_near, double[] x_new) {
        Collections.sort(X_near);
        Iterator<NearNode> ni = X_near.iterator();
        while (ni.hasNext()) {
            NearNode nearNode = ni.next();
            ni.remove();
            if (CollisionFree(nearNode.node.getState(), x_new)) {
                return nearNode.node;
            }
        }
        return null;
    }

    /** Add the node x_new to the tree, with an edge from x_min. */
    Node InsertNode(Node x_min, double[] x_new, KDNode<Node> rootNode) {
        Node newNode = new Node(x_new);
        LinkInterface newLink = Graph.newLink(_model, x_min, newNode);
        _bestLeaf_a = Graph.chooseBestPath(_model, _bestLeaf_a, newLink);
        KDTree.insert(_model, rootNode, newNode);
        return newNode;
    }

    /**
     * look through the nodes in X_near to see if any should be new children of
     * newNode.
     */
    void Rewire(List<NearNode> X_near, Node newNode) {
        ListIterator<NearNode> li = X_near.listIterator(X_near.size());
        while (li.hasPrevious()) {
            NearNode jn = li.previous();
            if (jn.node.getIncoming() != null) {
                if (Graph.rewire(_model, newNode, jn.node, jn.linkDist)) {
                    _bestLeaf_a = Graph.chooseBestPath(_model, _bestLeaf_a, newNode.getIncoming());
                }
            }
        }
    }

    /** Return all nodes in both trees. TODO: this is probably wrong */
    @Override
    public Iterable<Node> getNodes() {
        List<Node> allNodes = new ArrayList<Node>();
        allNodes.addAll(KDTree.values(_T_a));
        allNodes.addAll(KDTree.values(_T_b));
        return allNodes;
    }

    /**
     * the path distance may have been changed by rewiring. does this actually
     * matter? experiment says no.
     * TODO: remove this
     */
    public Path getFullBestPath() {
        Path bestPath = null;
        for (Map.Entry<Node, Node> entry : connections.entrySet()) {
            Path aPath = GeneratePath(entry.getKey(), entry.getValue());
            if (bestPath == null) {
                bestPath = aPath;
            } else {
                if (aPath.getDistance() < bestPath.getDistance())
                    bestPath = aPath;
            }
        }
        return bestPath;
    }

    @Override
    public Path getBestPath() {
        return _sigma_best;
    }

    // @Override
    // public Path getBestPath() {
    // LinkInterface link = _bestLeaf_a;
    // if (link == null) {
    // return null;
    // }
    // Node node = link.get_target();

    /**
     * Starting from leaf node, walk the parent links to accumulate
     * the full path, and reverse it, to return a path from root to node.
     * 
     * @param node leaf node
     */
    Path walkParents(Node node) {
        // Collect the states along the path (backwards)
        List<double[]> configs = new LinkedList<double[]>();
        // Since we're visiting all the nodes it's very cheap to verify the total
        // distance
        double totalDistance = 0;
        while (true) {
            configs.add(node.getState());
            LinkInterface incoming = node.getIncoming();
            if (incoming == null)
                break;
            totalDistance += incoming.get_linkDist();
            node = incoming.get_source();
        }
        // now we have the forwards list of states
        Collections.reverse(configs);

        return new Path(totalDistance, configs);
    }

    @Override
    public void setStepNo(int stepNo) {
        if (stepNo < 1)
            throw new IllegalArgumentException();
        this.stepNo = stepNo;
        this.radius = _gamma * Math.pow(
                Math.log(stepNo + 1) / (stepNo + 1),
                1.0 / _model.dimensions());
    }
}