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
import java.util.Random;
import java.util.function.BiFunction;

import org.team100.lib.graph.Graph;
import org.team100.lib.graph.LinkInterface;
import org.team100.lib.graph.LocalLink;
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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;

/**
 * RRT* version 5.
 * 
 * For the non-Euclidean spaces, state sampling doesn't seem to work
 * very well; there's no good way to find "nearby" states, and the
 * admissible fraction is tiny, especially with a weak control.
 * 
 * So instead, sample control space and apply to a random node.
 * 
 * maybe try
 */
public class RRTStar5<T extends KDModel & RobotModel> implements Solver {
    /**
     * probability of branching
     */
    private static final double BUSHINESS = 0.001;
    private final T _model;
    /** Initially, tree grown from initial, but is swapped repeatedly */
    private KDNode<Node> _T_a;
    /** Initially, tree grown from goal, but is swapped repeatedly */
    private KDNode<Node> _T_b;
    // private final Sample _sample;
    private final double _gamma;
    /** Lowest cost leaf leading to initial state. */
    // TODO: remove this
    private LinkInterface _bestLeaf_a;

    // mutable loop variables to make the loop code cleaner
    int stepNo;
    double radius;

    Path _sigma_best;
    Map<Node, Node> connections = new HashMap<Node, Node>();

    boolean bidirectional = false;

    Random random = new Random();
    private static final double MIN_U = -5;
    private static final double MAX_U = 5;
    // private static final double MIN_DT = 0.01;
    // private static final double MAX_DT = 0.25;
    private static final double DT = 0.1;
    private static final double l = 1; // length meter
    private static final double _g = 9.81; // gravity m/s/s
    private static final int MAX_CHILDREN = 1;
    private static final double BUFFER = 0.025;

    double[] min;
    double[] max;

    public RRTStar5(T model, Sample sample, double gamma) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        _model = model;
        _T_a = new KDNode<Node>(new Node(model.initial()));
        _T_b = new KDNode<Node>(new Node(model.goal()));
        // _sample = sample;
        _gamma = gamma;
        _bestLeaf_a = null;
        min = _model.getMin();
        max = _model.getMax();
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
        // System.out.println("step");
        int edges = 0;

        // the sample is now control-sampled, guaranteed feasible.

        // double[] x_rand = SampleFree();
        LocalLink randLink = SampleFree();
        // System.out.println("x_rand: " + Arrays.toString(x_rand));

        // KDNearNode<Node> x_nearest = Nearest(x_rand, _T_a);
        // KDNearNode<Node> x_nearest = Nearest(x_rand, _T_a);
        // if (x_nearest._nearest == null)
        // return edges;
        // System.out.println("x_nearest: " + x_nearest);

        // double[] x_new = Steer(x_nearest, x_rand);
        // double[] x_new = randLink.get_target().getState();
        // if (x_new == null)
        // return edges;
        // System.out.println("x_new: " + Arrays.toString(x_new));

        if (CollisionFree(randLink.get_source().getState(), randLink.get_target().getState())) {
            // X_near uses Euclidean metric, so do the distance over
            // actually for now don't use this near-search thing at all

            // List<NearNode> X_near = Near(x_new, _T_a);
            // if (X_near.isEmpty())
            // X_near.add(new NearNode(x_nearest._nearest, x_nearest._dist));
            // Node x_min = ChooseParent(X_near, x_new);

            // Node x_min = x_nearest._nearest;
            // if (x_min != null) {
            // if (_model.dist(x_min.getState(), x_new) < 0)
            // return edges;
            // Node newNode =

            Node newNode = InsertNode(randLink, _T_a);
            // System.out.println(newNode);
            // Rewire(X_near, newNode);
            edges += 1;

            if (bidirectional) {
                // is there a point in the other tree that is reachable from
                // the node we just inserted? reachable nodes are nearby in a Euclidean
                // sense, though most Euclidean nearby nodes are not reachable.
                // so start with a list of near nodes and test them one by one.
                //
                List<NearNode> X_near = Near(newNode.getState(), _T_b);
                for (NearNode nearNode : X_near) {

                }

                // KDNearNode<Node> x_conn = Nearest(x_new, _T_b);
                // Path sigma_new = Connect(newNode, x_conn, _T_b);
                // if (sigma_new != null) {
                // edges += 1;
                if (_sigma_best == null) {
                    // _sigma_best = sigma_new;
                } else {
                    // if (sigma_new.getDistance() < _sigma_best.getDistance()) {
                    // _sigma_best = sigma_new;
                    // }
                }
                // }
            }

            // }
        }
        if (bidirectional)
            SwapTrees();
        return edges;
    }

    public void SwapTrees() {
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
    // Path Connect(Node x_1, KDNearNode<Node> x_2, KDNode<Node> rootNode) {
    // // this is now just a filter
    // double[] x_new = Steer(x_2, x_1.getState());
    // if (x_new == null)
    // return null;
    // if (!same(x_new, x_1.getState()))
    // return null;
    // if (CollisionFree(x_2._nearest.getState(), x_1.getState())) {
    // // List<NearNode> X_near = Near(x_1.getState(), rootNode);
    // // if (X_near.isEmpty())
    // // X_near.add(new NearNode(x_2._nearest, x_2._dist));
    // // Node x_min = ChooseParent(X_near, x_1.getState());

    // // skip near-search for now since it isn't Euclidean
    // Node x_min = x_2._nearest;

    // if (x_min != null) {
    // Node newNode = InsertNode(x_min, x_1.getState(), rootNode);
    // connections.put(x_1, newNode);
    // // Rewire(X_near, newNode);
    // return GeneratePath(x_1, newNode);
    // }
    // }
    // return null;
    // }

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
        // don't include the same point twice
        states_2.remove(0);
        List<double[]> fullStates = new ArrayList<double[]>();
        fullStates.addAll(p_1.getStates());
        fullStates.addAll(states_2);
        return new Path(p_1.getDistance() + p_2.getDistance(), fullStates);
    }

    boolean CollisionFree(double[] from, double[] to) {
        return _model.link(from, to);
    }

    /**
     * Applies random control to random tree node. If the node
     * has a parent, just continue the same way as that node.
     */
    LocalLink SampleFree() {
        // run backwards if the tree root is the goal
        boolean timeForward = same(_T_a.getValue().getState(), _model.initial());

        while (true) {
            // applied to a random point in the tree
            List<Node> nodes = getNodes();
            int nodect = nodes.size();
            int nodeidx = random.nextInt(nodect);
            Node node_rand = nodes.get(nodeidx);
            // persuade the tree to be longer
            if (node_rand.getOutgoingCount() >= MAX_CHILDREN) {
                // maybe add anyway?
                if (random.nextDouble() > BUSHINESS)
                    continue;
            }

            double x_nearest1 = node_rand.getState()[0];
            double x_nearest2 = node_rand.getState()[1];

            double x1dot = x_nearest2;
            // random control
            double u = MIN_U + (MAX_U - MIN_U) * random.nextDouble();
            // u = 0;
            double x2dot = -1 * _g * Math.sin(x_nearest1) / l + u;
            double dt = DT;
            if (!timeForward)
                dt *= -1.0;

            double x_new1;
            double x_new2;

            // accurate integration is absolutely required here,
            // otherwise the dynamics are wrong
            BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = (xx, uu) -> {
                double xx1 = xx.get(0, 0);
                double xx2 = xx.get(1, 0);
                double xx1dot = xx2;
                double xx2dot = -1 * _g * Math.sin(xx1) / l + uu.get(0, 0);
                Matrix<N2, N1> result = new Matrix<>(Nat.N2(), Nat.N1());
                result.set(0, 0, xx1dot);
                result.set(1, 0, xx2dot);
                return result;
            };

            Matrix<N2, N1> xxx = new Matrix<>(Nat.N2(), Nat.N1());
            xxx.set(0, 0, x_nearest1);
            xxx.set(1, 0, x_nearest2);
            Matrix<N1, N1> uuu = new Matrix<>(Nat.N1(), Nat.N1());
            uuu.set(0, 0, u);
            Matrix<N2, N1> newxxx = NumericalIntegration.rk4(f, xxx, uuu, dt);

            // if (timeForward) {
            // x_new1 = x_nearest1 + x1dot * dt + 0.5 * x2dot * dt * dt;
            // x_new2 = x_nearest2 + x2dot * dt;
            // } else {
            // x_new1 = x_nearest1 - x1dot * dt - 0.5 * x2dot * dt * dt;
            // x_new2 = x_nearest2 - x2dot * dt;
            // }
            x_new1 = newxxx.get(0, 0);
            x_new2 = newxxx.get(1, 0);

            // reject samples off the edge of the world
            // TODO: this is actually a cylindrical space, so make it so
            if (x_new1 < min[0] || x_new1 > max[0] || x_new2 < min[1] || x_new2 > max[1]) {
                System.out.printf(
                        "reject out of bounds [%5.3f %5.3f] u %5.3f\n",
                        x_new1, x_new2, u);
                continue;
            }

            // System.out.printf("sample from [%5.3f %5.3f] to [%5.3f %5.3f] xdot [%5.3f
            // %5.3f] dt %5.3f u %5.3f\n",
            // x_nearest1, x_nearest2, x_new1, x_new2, x1dot, x2dot, DT, u);

            double[] newConfig = new double[] { x_new1, x_new2 };

            double[] dx_new = new double[] { x_new1 - x_nearest1, x_new2 - x_nearest2 };
            if (node_rand.getIncoming() != null) {
                // continue in the same direction as the incoming,
                // to avoid clumping
                // could use the same "u" (or nearly same) here but
                // we want to allow the bang-bang deceleration case
                LinkInterface incoming = node_rand.getIncoming();
                double[] incoming_dx_new = new double[] {
                        incoming.get_target().getState()[0] - incoming.get_source().getState()[0],
                        incoming.get_target().getState()[1] - incoming.get_source().getState()[1]
                };
                double dot = incoming_dx_new[0] * dx_new[0]
                        + incoming_dx_new[1] * dx_new[1];
                if (dot < 0) {
                    System.out.printf(
                            "reject dot parent [%5.3f %5.3f] node [%5.3f %5.3f] to [%5.3f %5.3f] u %5.3f dot %5.3f\n",
                            incoming.get_source().getState()[0], incoming.get_source().getState()[1],
                            x_nearest1, x_nearest2,
                            x_new1, x_new2,
                            u, dot);
                    continue;
                }

                // reject the new node if it's too close to any others
                // for now just use Euclidean distance.
                // note this will find the parent so make sure the step
                // size is larger than the buffer size
                KDNearNode<Node> n = KDTree.nearest(_model, _T_a, newConfig);
                if (n != null) {

                    double newDist = Math.sqrt(Math.pow(x_new1 - n._nearest.getState()[0], 2) +
                            Math.pow(x_new2 - n._nearest.getState()[1], 2));
                    if (newDist < BUFFER) {
                        System.out.printf(
                                "reject conflict from [%5.3f %5.3f] to [%5.3f %5.3f] old [%5.3f %5.3f] d %5.3f\n",
                                x_nearest1, x_nearest2,
                                x_new1, x_new2,
                                n._nearest.getState()[0], n._nearest.getState()[1],
                                newDist);
                        continue;
                    }
                }

            }

            // double[] newConfig = _sample.get();
            if (_model.clear(newConfig)) {
                System.out.printf(
                        "found new [%5.3f %5.3f] u %5.3f dt %5.3f\n",
                        x_new1, x_new2, u, dt);
                return new LocalLink(node_rand, new Node(newConfig), DT);
            }
            // return newConfig;
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
        // for now just do it the dumb way
        List<Node> nodes = KDTree.values(rootNode);
        Node bestNode = null;
        double bestDistance = Double.MAX_VALUE;
        for (Node node : nodes) {
            double d = _model.dist(node.getState(), x_rand);
            if (d <= 0)
                continue;
            if (d < bestDistance) {
                bestDistance = d;
                bestNode = node;
            }
        }
        return new KDNearNode<Node>(bestDistance, bestNode);
        // return KDTree.nearest(_model, rootNode, x_rand);
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
        // _model.setRadius(radius);
        _model.setRadius(same(_T_a.getValue().getState(), _model.initial()) ? 1 : -1);
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
    Node InsertNode(LocalLink link, KDNode<Node> rootNode) {
        System.out.printf("newLink from [%5.3f %5.3f] to [%5.3f %5.3f] d %5.3f\n",
                link.get_source().getState()[0], link.get_source().getState()[1],
                link.get_target().getState()[0], link.get_target().getState()[1],
                link.get_linkDist());
        LinkInterface newLink = Graph.newLink(link.get_source(), link.get_target(), link.get_linkDist());
        _bestLeaf_a = Graph.chooseBestPath(_model, _bestLeaf_a, newLink);
        KDTree.insert(_model, rootNode, link.get_target());
        return link.get_target();
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
    public List<Node> getNodes() {
        List<Node> allNodes = new ArrayList<Node>();
        allNodes.addAll(KDTree.values(_T_a));
        if (bidirectional)
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

    static boolean same(double[] a, double[] b) {
        if (a.length != b.length)
            return false;
        for (int i = 0; i < a.length; ++i) {
            if (Math.abs(a[i] - b[i]) > 0.0001)
                return false;
        }
        return true;
    }
}