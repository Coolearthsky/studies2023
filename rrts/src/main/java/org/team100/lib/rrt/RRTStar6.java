package org.team100.lib.rrt;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.Random;
import java.util.Set;
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
import org.team100.lib.math.ShootingSolver;
import org.team100.lib.planner.RobotModel;
import org.team100.lib.planner.Solver;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;
import org.team100.lib.util.Util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;

/**
 * RRT* version 6
 * 
 * This is the full-state field, so 4d altogether.
 */
public class RRTStar6<T extends KDModel & RobotModel> implements Solver {
    private static final boolean DEBUG = true;

    /**
     * probability of branching
     */
    private static final double BUSHINESS = 0.5;
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

    boolean bidirectional = true;

    Random random = new Random();
    private static final double MAX_U = 2.5;
    // private static final double MIN_DT = 0.01;
    // private static final double MAX_DT = 0.25;
    private static final double DT = 0.5;
    private static final double l = 1; // length meter
    private static final double _g = 9.81; // gravity m/s/s
    private static final int MAX_CHILDREN = 1;
    private static final double BUFFER = 0.025;

    double[] min;
    double[] max;

    ShootingSolver<N4, N2> solver = new ShootingSolver<>(VecBuilder.fill(MAX_U, MAX_U), DT, 20);

    public RRTStar6(T model, Sample sample, double gamma) {
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

    // The top level is just a 2d double-integrator.
    BiFunction<Matrix<N4, N1>, Matrix<N2, N1>, Matrix<N4, N1>> f = (x, u) -> {
        double x1 = x.get(0, 0);
        double x2 = x.get(1, 0);
        double y1 = x.get(2, 0);
        double y2 = x.get(3, 0);
        double ux = u.get(0, 0);
        double uy = u.get(1, 0);
        double x1dot = x2;
        double x2dot = ux;
        double y1dot = y2;
        double y2dot = uy;
        Matrix<N4, N1> result = new Matrix<>(Nat.N4(), Nat.N1());
        result.set(0, 0, x1dot);
        result.set(1, 0, x2dot);
        result.set(2, 0, y1dot);
        result.set(3, 0, y2dot);
        return result;
    };

    /**
     * Note this isn't quite the same as https://arxiv.org/pdf/1703.08944.pdf
     * because it doesn't use Extend, so it doesn't try to connect unless
     * a new node is actually inserted.
     * 
     * @return true if a new sample was added.
     */
    @Override
    public int step() {
        if (DEBUG)
            System.out.println("step");
        int edges = 0;

        // the sample is now control-sampled, guaranteed feasible.

        // double[] x_rand = SampleFree();
        LocalLink randLink = SampleFree();
        if (DEBUG)
            System.out.println("randLink: " + randLink);

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

            // new node in "this" tree
            Node newNode = InsertNode(randLink, _T_a);
            if (DEBUG)
                System.out.println("NEW NODE " + newNode);
            List<NearNode> X_nearA = Near(newNode.getState(), _T_a);
            Rewire(X_nearA, newNode);

            edges += 1;

            if (bidirectional) {
                // is there a point in the other tree that is reachable from
                // the node we just inserted? reachable nodes are nearby in a Euclidean
                // sense, though most Euclidean nearby nodes are not reachable.
                // so start with a list of near nodes and test them one by one.

                // near nodes in the other tree:
                List<NearNode> X_near = Near(newNode.getState(), _T_b);
                Matrix<N4, N1> x1 = VecBuilder.fill(
                        newNode.getState()[0],
                        newNode.getState()[1],
                        newNode.getState()[2],
                        newNode.getState()[3]);
                for (NearNode nearNode : X_near) {
                    // one near node in the other tree
                    Matrix<N4, N1> x2 = VecBuilder.fill(
                            nearNode.node.getState()[0],
                            nearNode.node.getState()[1],
                            nearNode.node.getState()[2],
                            nearNode.node.getState()[3]);
                    ShootingSolver<N4, N2>.Solution sol = solver.solve(Nat.N4(), Nat.N2(), f, x1, x2);
                    if (sol != null) {
                        // there's a route from x1 aka newnode (in a) to x2 aka nearnode (in b)
                        // System.out.printf("FOUND feasible link x1: %s x2: %s sol: %s\n",
                        // Util.matStr(x1), Util.matStr(x2), sol);
                        // TODO: do something with the solution u value
                        // add a node in a corresponding to the near node in b
                        LocalLink newInA = new LocalLink(newNode, new Node(nearNode.node.getState()), sol.dt);
                        // LocalLink link = new LocalLink(newNode, nearNode.node, sol.dt);
                        Node newNewNode = InsertNode(newInA, _T_a);
                        connections.put(newNode, nearNode.node);
                        // Rewire(X_near, newNode);
                        // return GeneratePath(x_1, newNode);
                        // create a path that traverses the new link.
                        Path p = GeneratePath(newNewNode, nearNode.node);
                        // System.out.println("PATH " + p);
                        // Path p = null;
                        if (_sigma_best == null) {
                            _sigma_best = p;
                        } else {
                            if (p.getDistance() < _sigma_best.getDistance()) {
                                _sigma_best = p;
                            }
                        }
                        // don't need more than one feasible link
                        break;
                    }
                }

                // KDNearNode<Node> x_conn = Nearest(x_new, _T_b);
                // Path sigma_new = Connect(newNode, x_conn, _T_b);
                // if (sigma_new != null) {
                // edges += 1;
                // if (_sigma_best == null) {
                // _sigma_best = sigma_new;
                // } else {
                // if (sigma_new.getDistance() < _sigma_best.getDistance()) {
                // _sigma_best = sigma_new;
                // }
                // }
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
        Path p_1 = walkParents(new HashSet<Node>(), x_1);
        // System.out.println("p1 " + p_1);
        Path p_2 = walkParents(new HashSet<Node>(), x_2);
        // System.out.println("p2 " + p_2);
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
     * Applies random control to random tree node *in T_a.* If the node
     * has a parent, just continue the same way as that node.
     */
    LocalLink SampleFree() {
        // run backwards if the tree root is the goal
        boolean timeForward = same(_T_a.getValue().getState(), _model.initial());
        if (DEBUG)
            System.out.println("SampleFree");
        while (true) {
            // if (DEBUG)
            // System.out.println("sample");
            // applied to a random point in the tree
            // List<Node> nodes = getNodes();
            List<Node> nodes = KDTree.values(_T_a);
            int nodect = nodes.size();
            int nodeidx = random.nextInt(nodect);
            Node node_rand = nodes.get(nodeidx);
            // persuade the tree to be longer
            if (node_rand.getOutgoingCount() >= MAX_CHILDREN) {
                // maybe add anyway?
                if (random.nextDouble() > BUSHINESS) {
                    // if (DEBUG)
                    // System.out.println("skip node with too many children");
                    continue;
                }
            }

            double x_nearest1 = node_rand.getState()[0];
            double x_nearest2 = node_rand.getState()[1];
            double x_nearest3 = node_rand.getState()[2];
            double x_nearest4 = node_rand.getState()[3];

            // TODO: add randomness to dt
            double dt = DT;
            if (!timeForward)
                dt *= -1.0;

            Matrix<N4, N1> xxx = new Matrix<>(Nat.N4(), Nat.N1());
            xxx.set(0, 0, x_nearest1);
            xxx.set(1, 0, x_nearest2);
            xxx.set(2, 0, x_nearest3);
            xxx.set(3, 0, x_nearest4);

            Matrix<N2, N1> uuu = new Matrix<>(Nat.N2(), Nat.N1());
            uuu.set(0, 0, u_rand());
            uuu.set(1, 0, u_rand());

            // if (timeForward) {
            // x_new1 = x_nearest1 + x1dot * dt + 0.5 * x2dot * dt * dt;
            // x_new2 = x_nearest2 + x2dot * dt;
            // } else {
            // x_new1 = x_nearest1 - x1dot * dt - 0.5 * x2dot * dt * dt;
            // x_new2 = x_nearest2 - x2dot * dt;
            // }

            Matrix<N4, N1> newxxx = NumericalIntegration.rk4(f, xxx, uuu, dt);
            double x_new1 = newxxx.get(0, 0);
            double x_new2 = newxxx.get(1, 0);
            double x_new3 = newxxx.get(2, 0);
            double x_new4 = newxxx.get(3, 0);

            if (DEBUG)
                System.out.printf("integrated to get %s\n", Util.matStr(newxxx));

            // reject samples off the edge of the world
            // TODO: this is actually a cylindrical space, so make it so
            if (x_new1 < min[0] || x_new1 > max[0]
                    || x_new2 < min[1] || x_new2 > max[1]
                    || x_new3 < min[2] || x_new3 > max[2]
                    || x_new4 < min[3] || x_new4 > max[3]

            ) {
                if (DEBUG)
                    System.out.printf("reject out of bounds %s u %s\n",
                            Util.matStr(newxxx), Util.matStr(uuu));
                continue;
            }

            // System.out.printf("sample from [%5.3f %5.3f] to [%5.3f %5.3f] xdot [%5.3f
            // %5.3f] dt %5.3f u %5.3f\n",
            // x_nearest1, x_nearest2, x_new1, x_new2, x1dot, x2dot, DT, u);

            double[] newConfig = new double[] { x_new1, x_new2, x_new3, x_new4 };

            double[] dx_new = new double[] {
                    x_new1 - x_nearest1,
                    x_new2 - x_nearest2,
                    x_new3 - x_nearest3,
                    x_new4 - x_nearest4
            };
            if (node_rand.getIncoming() != null) {
                // continue in the same direction as the incoming,
                // to avoid clumping
                // could use the same "u" (or nearly same) here but
                // we want to allow the bang-bang deceleration case
                LinkInterface incoming = node_rand.getIncoming();
                double[] incoming_dx_new = new double[] {
                        incoming.get_target().getState()[0] - incoming.get_source().getState()[0],
                        incoming.get_target().getState()[1] - incoming.get_source().getState()[1],
                        incoming.get_target().getState()[2] - incoming.get_source().getState()[2],
                        incoming.get_target().getState()[3] - incoming.get_source().getState()[3]

                };
                double dot = incoming_dx_new[0] * dx_new[0]
                        + incoming_dx_new[1] * dx_new[1]
                        + incoming_dx_new[2] * dx_new[2]
                        + incoming_dx_new[3] * dx_new[3];
                if (dot < 0) {
                    if (DEBUG)
                        System.out.printf(
                                "reject dot parent [%5.3f %5.3f %5.3f %5.3f] node %s to %s u %s dot %5.3f\n",
                                incoming.get_source().getState()[0], incoming.get_source().getState()[1],
                                incoming.get_source().getState()[2], incoming.get_source().getState()[3],
                                Util.matStr(xxx),
                                Util.matStr(newxxx),
                                Util.matStr(uuu), dot);
                    continue;
                }

                // reject the new node if it's too close to any others
                // for now just use Euclidean distance.
                // note this will find the parent so make sure the step
                // size is larger than the buffer size
                KDNearNode<Node> n = KDTree.nearest(_model, _T_a, newConfig);
                if (n != null) {
                    // look only at spatial dimensions; it's ok for there to be lots of
                    // points at the same velocity.
                    double newDist = Math.sqrt(Math.pow(x_new1 - n._nearest.getState()[0], 2) +
                            Math.pow(x_new3 - n._nearest.getState()[2], 2));
                    if (newDist < BUFFER) {
                        if (DEBUG)
                            System.out.printf(
                                    "reject conflict from %s to %s old [%5.3f %5.3f %5.3f %5.3f] d %5.3f\n",
                                    Util.matStr(xxx),
                                    Util.matStr(newxxx),
                                    n._nearest.getState()[0], n._nearest.getState()[1],
                                    n._nearest.getState()[2], n._nearest.getState()[3],
                                    newDist);
                        continue;
                    }
                }

            }

            // double[] newConfig = _sample.get();
            if (_model.clear(newConfig)) {
                if (DEBUG)
                    System.out.printf(
                            "found new %s u %s dt %5.3f\n",
                            Util.matStr(newxxx), Util.matStr(uuu), dt);
                return new LocalLink(node_rand, new Node(newConfig), DT);
            }
            if (DEBUG)
                System.out.println("not clear");
            // return newConfig;
        }
    }

    /**
     * random control
     * 
     * TODO: shape the sampling to look more "bang bang" like.
     */
    double u_rand() {
        double u = MAX_U + random.nextGaussian(0, MAX_U / 10);
        if (u > MAX_U)
            u = -2.0 * MAX_U + u;
        u = Math.max(-1.0 * MAX_U, u);
        u = Math.min(MAX_U, u);
        // double u = 0;
        // System.out.println("U " + u);
        return u;
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

    /** Add the node link.target to the tree, with an edge from source to target. */
    Node InsertNode(LocalLink link, KDNode<Node> rootNode) {
        // System.out.printf("newLink from [%5.3f %5.3f] to [%5.3f %5.3f] d %5.3f\n",
        // link.get_source().getState()[0], link.get_source().getState()[1],
        // link.get_target().getState()[0], link.get_target().getState()[1],
        // link.get_linkDist());
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
                Matrix<N4, N1> x1 = VecBuilder.fill(
                        newNode.getState()[0],
                        newNode.getState()[1],
                        newNode.getState()[2],
                        newNode.getState()[3]);
                Matrix<N4, N1> x2 = VecBuilder.fill(
                        jn.node.getState()[0],
                        jn.node.getState()[1],
                        jn.node.getState()[2],
                        jn.node.getState()[3]);
                ShootingSolver<N4, N2>.Solution sol = solver.solve(Nat.N4(), Nat.N2(), f, x1, x2);
                if (sol != null) {
                    if (Graph.rewire(_model, newNode, jn.node, sol.dt)) {
                        // System.out.println("REWIRED");
                        _bestLeaf_a = Graph.chooseBestPath(_model, _bestLeaf_a, newNode.getIncoming());
                    }

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
    Path walkParents(Set<Node> visited, Node node) {
        // Collect the states along the path (backwards)
        List<double[]> configs = new LinkedList<double[]>();
        // Since we're visiting all the nodes it's very cheap to verify the total
        // distance
        double totalDistance = 0;
        while (true) {
            // System.out.printf("walking node %s\n", node);
            if (visited.contains(node)) {
                System.out.println("found a cycle");
                throw new IllegalArgumentException();
            }
            visited.add(node);
            // System.out.println("walk");
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