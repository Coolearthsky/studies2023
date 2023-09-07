package org.team100.lib.rrt;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
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
import org.team100.lib.random.MersenneTwister;
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
 * RRT* version 7
 * 
 * full-state 4d field, following the bang-bang rrt paper.
 * 
 * https://arxiv.org/pdf/2210.01744.pdf
 * 
 * alpha = sample from state
 * x_n = nearest neighbor
 * find a path to the steer to sample using max U
 * make the times of each axis match, respecting the gap if it exists
 * same routine for rewiring i guess?
 * 
 * 
 * 
 * for rewiring and connecting, use a linear solver.
 */
public class RRTStar7<T extends KDModel<N4> & RobotModel<N4>> implements Solver<N4> {
    private static final boolean DEBUG = false;
    private static final double MAX_U = 2.5;
    private static final double DT = 0.6;
    private static final int MAX_CHILDREN = 1;
    private static final double BUFFER = 0.3;
    /** for testing */
    private static final boolean BIDIRECTIONAL = true;
    /** probability of branching */
    private static final double BUSHINESS = 0.2;

    private final T _model;
    private final Sample<N4> _sample;
    private final double _gamma;
    private final Random random = new MersenneTwister(new Random().nextInt());
    private final ShootingSolver<N4, N2> solver = new ShootingSolver<>(VecBuilder.fill(MAX_U, MAX_U), DT, 20);
    private final Matrix<N4, N1> min;
    private final Matrix<N4, N1> max;

    /** Initially, tree grown from initial, but is swapped repeatedly */
    private KDNode<Node<N4>> _T_a;
    /** Initially, tree grown from goal, but is swapped repeatedly */
    private KDNode<Node<N4>> _T_b;

    // mutable loop variables to make the loop code cleaner
    private int stepNo;
    private double radius;
    private Path<N4> _sigma_best;

    public RRTStar7(T model, Sample<N4> sample, double gamma, KDNode<Node<N4>> T_a, KDNode<Node<N4>> T_b) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        _model = model;
        _sample = sample;
        _T_a = T_a;
        _T_b = T_b;
        _gamma = gamma;
        min = _model.getMin();
        max = _model.getMax();
    }

    /** The top level is just a 2d double-integrator. */
    BiFunction<Matrix<N4, N1>, Matrix<N2, N1>, Matrix<N4, N1>> f = (x, u) -> {
        double x2 = x.get(1, 0);
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

        boolean timeForward = same(_T_a.getValue().getState(), _model.initial());

        // alpha
        Matrix<N4, N1> x_rand = SampleState();

        // x_n
        KDNearNode<Node<N4>> x_nearest = Nearest(x_rand, _T_a, timeForward);

        LocalLink<N4> randLink = SampleFree(timeForward);
        if (DEBUG)
            System.out.println("randLink: " + randLink);

        if (CollisionFree(randLink.get_source().getState(), randLink.get_target().getState())) {
            // new node in "this" tree
            Node<N4> newNode = InsertNode(randLink, _T_a);
            if (DEBUG)
                System.out.println("NEW NODE " + newNode);
            List<NearNode<N4>> X_nearA = Near(newNode.getState(), _T_a);
            Rewire(X_nearA, newNode, timeForward);

            edges += 1;

            if (BIDIRECTIONAL) {
                // is there a point in the other tree that is reachable from
                // the node we just inserted? reachable nodes are nearby in a Euclidean
                // sense, though most Euclidean nearby nodes are not reachable.
                // so start with a list of near nodes and test them one by one.

                // near nodes in the other tree:
                List<NearNode<N4>> X_near = Near(newNode.getState(), _T_b);
                Matrix<N4, N1> x1 = newNode.getState();
                for (NearNode<N4> nearNode : X_near) {
                    // one near node in the other tree
                    Matrix<N4, N1> x2 = nearNode.node.getState();

                    ShootingSolver<N4, N2>.Solution sol = solver.solve(Nat.N4(), Nat.N2(), f, x1, x2, timeForward);
                    if (sol != null) {
                        // there's a route from x1 aka newnode (in a) to x2 aka nearnode (in b)
                        if (DEBUG)
                            System.out.printf("FOUND feasible link x1: %s x2: %s sol: %s\n",
                                    Util.matStr(x1), Util.matStr(x2), sol);
                        // TODO: do something with the solution u value
                        // add a node in a corresponding to the near node in b
                        LocalLink<N4> newInA = new LocalLink<>(newNode, new Node<>(nearNode.node.getState()),
                                Math.abs(sol.dt));
                        Node<N4> newNewNode = InsertNode(newInA, _T_a);
                        Rewire(X_near, newNewNode, timeForward);
                        // create a path that traverses the new link.
                        Path<N4> p = GeneratePath(newNewNode, nearNode.node);
                        if (DEBUG)
                            System.out.println("PATH " + p);
                        if (_sigma_best == null) {
                            System.out.printf("first path distance %7.3f\n", p.getDistance());
                            _sigma_best = p;
                        } else {
                            if (p.getDistance() < _sigma_best.getDistance()) {
                                System.out.printf("new best path distance %7.3f\n", p.getDistance());
                                _sigma_best = p;
                            }
                        }
                        // don't need more than one feasible link
                        break;
                    }
                }
            }
        }
        if (BIDIRECTIONAL)
            SwapTrees();
        return edges;
    }

    public void SwapTrees() {
        KDNode<Node<N4>> tmp = _T_a;
        _T_a = _T_b;
        _T_b = tmp;
    }

    /**
     * the parameters describe a link between initial and goal trees, the same
     * state in both cases.
     */
    Path<N4> GeneratePath(Node<N4> x_1, Node<N4> x_2) {
        if (!x_1.getState().isEqual(x_2.getState(), 0.001))
            throw new IllegalArgumentException(
                    "x1 " + x_1.getState().toString() + " != x2 " + x_2.getState().toString());

        Path<N4> p_1 = walkParents(new HashSet<>(), x_1);
        if (DEBUG)
            System.out.println("p1 " + p_1);
        Path<N4> p_2 = walkParents(new HashSet<>(), x_2);
        if (DEBUG)
            System.out.println("p2 " + p_2);
        List<Matrix<N4, N1>> states_2 = p_2.getStatesA();
        Collections.reverse(states_2);
        // don't include the same point twice
        states_2.remove(0);
        return new Path<>(p_1.getDistance() + p_2.getDistance(), p_1.getStatesA(), states_2);
    }

    boolean CollisionFree(Matrix<N4, N1> from, Matrix<N4, N1> to) {
        return _model.link(from, to);
    }

    /** Sample the free state. */
    Matrix<N4, N1> SampleState() {
        while (true) {
            Matrix<N4, N1> newConfig = _sample.get();
            if (_model.clear(newConfig))
                return newConfig;
        }
    }

    /**
     * Return the nearest node in the tree.
     * 
     * This works by finding a set of Euclidean-near nodes from the KD tree,
     * rescoring all of them, and returning the lowest-scored one.
     * 
     * note this scores each of the double-integrators seprately; the state is
     * (x xdot y ydot).
     * 
     * for now, it just takes the max time, but
     * TODO: respect waiting time
     * 
     * @param x_rand   the random sample
     * @param rootNode the tree to look through
     */
    KDNearNode<Node<N4>> Nearest(Matrix<N4, N1> x_rand, KDNode<Node<N4>> rootNode, boolean timeForward) {
        // For now, use the Near function, which uses the "radius". Maybe
        // it would be better to choose top-N-near, or use a different radius,
        // or whatever.
        ArrayList<NearNode<N4>> nodes = Near(x_rand, rootNode);
        double bestT = Double.MAX_VALUE;
        Node<N4> bestNode = null;
        for (NearNode<N4> node : nodes) {
            // rescore each node.
            double tx;
            double ty;
            if (timeForward) {
                // from x_rand to node
                tx = tSwitch(
                        x_rand.get(0, 0), x_rand.get(1, 0),
                        node.node.getState().get(0, 0), node.node.getState().get(1, 0),
                        MAX_U);
                ty = tSwitch(
                        x_rand.get(2, 0), x_rand.get(3, 0),
                        node.node.getState().get(2, 0), node.node.getState().get(3, 0),
                        MAX_U);
            } else {
                // from node to x_rand
                tx = tSwitch(
                        node.node.getState().get(0, 0), node.node.getState().get(1, 0),
                        x_rand.get(0, 0), x_rand.get(1, 0),
                        MAX_U);
                ty = tSwitch(
                        node.node.getState().get(2, 0), node.node.getState().get(3, 0),
                        x_rand.get(2, 0), x_rand.get(3, 0),
                        MAX_U);
            }
            double t = Math.max(tx, ty);
            if (t < bestT) {
                bestT = t;
                bestNode = node.node;
            }
        }
        return new KDNearNode<>(bestT, bestNode);
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
    Matrix<N4, N1> Steer(KDNearNode<Node<N4>> x_nearest, Matrix<N4, N1> x_rand) {
        _model.setStepNo(stepNo);
        // _model.setRadius(radius);
        _model.setRadius(same(_T_a.getValue().getState(), _model.initial()) ? 1 : -1);
        return _model.steer(x_nearest, x_rand);
    }

    /**
     * Applies random control to random tree node *in T_a.* If the node
     * has a parent, just continue the same way as that node.
     */
    LocalLink<N4> SampleFree(boolean timeForward) {
        // run backwards if the tree root is the goal
        if (DEBUG)
            System.out.println("SampleFree");
        while (true) {
            if (DEBUG)
                System.out.println("sample");
            // applied to a random point in the tree
            List<Node<N4>> nodes = KDTree.values(_T_a);
            int nodect = nodes.size();
            int nodeidx = random.nextInt(nodect);
            Node<N4> node_rand = nodes.get(nodeidx);
            // persuade the tree to be longer
            if (node_rand.getOutgoingCount() >= MAX_CHILDREN) {
                // maybe add anyway?
                if (random.nextDouble() > BUSHINESS) {
                    if (DEBUG)
                        System.out.println("skip node with too many children");
                    continue;
                }
            }

            double x_nearest1 = node_rand.getState().get(0, 0);
            double x_nearest2 = node_rand.getState().get(1, 0);
            double x_nearest3 = node_rand.getState().get(2, 0);
            double x_nearest4 = node_rand.getState().get(3, 0);

            Matrix<N4, N1> xxx = new Matrix<>(Nat.N4(), Nat.N1());
            xxx.set(0, 0, x_nearest1);
            xxx.set(1, 0, x_nearest2);
            xxx.set(2, 0, x_nearest3);
            xxx.set(3, 0, x_nearest4);

            Matrix<N2, N1> uuu = new Matrix<>(Nat.N2(), Nat.N1());
            double azimuth = 2 * Math.PI * random.nextDouble();
            uuu.set(0, 0, MAX_U * Math.cos(azimuth));
            uuu.set(1, 0, MAX_U * Math.sin(azimuth));

            double dt = DT * random.nextDouble();
            // maybe integrate backwards :-)
            if (!timeForward)
                dt *= -1.0;
            Matrix<N4, N1> newxxx = NumericalIntegration.rk4(f, xxx, uuu, dt);
            double x_new1 = newxxx.get(0, 0);
            double x_new2 = newxxx.get(1, 0);
            double x_new3 = newxxx.get(2, 0);
            double x_new4 = newxxx.get(3, 0);

            if (DEBUG)
                System.out.printf("integrated to get %s\n", Util.matStr(newxxx));

            // reject samples off the edge of the world
            // TODO: this is actually a cylindrical space, so make it so
            if (x_new1 < min.get(0, 0) || x_new1 > max.get(0, 0)
                    || x_new2 < min.get(1, 0) || x_new2 > max.get(1, 0)
                    || x_new3 < min.get(2, 0) || x_new3 > max.get(2, 0)
                    || x_new4 < min.get(3, 0) || x_new4 > max.get(3, 0)

            ) {
                if (DEBUG)
                    System.out.printf("reject out of bounds %s u %s\n",
                            Util.matStr(newxxx), Util.matStr(uuu));
                continue;
            }

            Matrix<N4, N1> newConfig = newxxx;

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
                LinkInterface<N4> incoming = node_rand.getIncoming();
                double[] incoming_dx_new = new double[] {
                        incoming.get_target().getState().get(0, 0) - incoming.get_source().getState().get(0, 0),
                        incoming.get_target().getState().get(1, 0) - incoming.get_source().getState().get(1, 0),
                        incoming.get_target().getState().get(2, 0) - incoming.get_source().getState().get(2, 0),
                        incoming.get_target().getState().get(3, 0) - incoming.get_source().getState().get(3, 0)

                };
                double dot = incoming_dx_new[0] * dx_new[0]
                        + incoming_dx_new[1] * dx_new[1]
                        + incoming_dx_new[2] * dx_new[2]
                        + incoming_dx_new[3] * dx_new[3];
                if (dot < 0) {
                    if (DEBUG)
                        System.out.printf(
                                "reject dot parent [%5.3f %5.3f %5.3f %5.3f] node %s to %s u %s dot %5.3f\n",
                                incoming.get_source().getState().get(0, 0), incoming.get_source().getState().get(1, 0),
                                incoming.get_source().getState().get(2, 0), incoming.get_source().getState().get(3, 0),
                                Util.matStr(xxx),
                                Util.matStr(newxxx),
                                Util.matStr(uuu), dot);
                    continue;
                }

                // reject the new node if it's too close to any others
                // for now just use Euclidean distance.
                // note this will find the parent so make sure the step
                // size is larger than the buffer size
                KDNearNode<Node<N4>> n = KDTree.nearest(_model, _T_a, newConfig);
                if (n != null) {
                    // look only at spatial dimensions; it's ok for there to be lots of
                    // points at the same velocity.
                    double newDist = Math.sqrt(Math.pow(x_new1 - n._nearest.getState().get(0, 0), 2) +
                            Math.pow(x_new3 - n._nearest.getState().get(2, 0), 2));
                    if (newDist < BUFFER) {
                        if (DEBUG)
                            System.out.printf(
                                    "reject conflict from %s to %s old [%5.3f %5.3f %5.3f %5.3f] d %5.3f\n",
                                    Util.matStr(xxx),
                                    Util.matStr(newxxx),
                                    n._nearest.getState().get(0, 0), n._nearest.getState().get(1, 0),
                                    n._nearest.getState().get(2, 0), n._nearest.getState().get(3, 0),
                                    newDist);
                        continue;
                    }
                }

            }

            if (_model.clear(newConfig)) {
                if (DEBUG)
                    System.out.printf(
                            "found new %s u %s dt %5.3f\n",
                            Util.matStr(newxxx), Util.matStr(uuu), dt);
                // note abs() due to (sometimes) time reversal
                return new LocalLink<>(node_rand, new Node<>(newConfig), Math.abs(dt));
            }
            if (DEBUG)
                System.out.println("not clear");
        }
    }

    /**
     * not literally to the right of initial state, but to the right of the
     * switching surface containing the initial state. the paper has a different
     * way of expressing this which i think might be wrong.
     * https://arxiv.org/pdf/2210.01744.pdf
     * 
     * The params are primitives because we kinda pick them out of the N4 state.
     * 
     * @param i    initial position
     * @param idot initial velocity
     * @param g    goal position
     * @param gdot goal velocity
     */
    public static boolean goalRight(double i, double idot, double g, double gdot, double umax) {
        // intercept of umax parabola intersecting x1
        double c_I_plus = c_plus(i, idot, umax);
        // intercept of umin parabola intersecting x1
        double c_I_minus = c_minus(i, idot, umax);
        // intercept of umax parabola intersecting x2
        double c_G_plus = c_plus(g, gdot, umax);
        // intercept of umin parabola intersecting x2
        double c_G_minus = c_minus(g, gdot, umax);

        if (gdot > idot) {
            if (c_G_plus > c_I_plus) {
                return true;
            }
            return false;
        }
        if (c_G_minus > c_I_minus) {
            return true;
        }
        return false;
    }

    /**
     * for a single 2d double-integrator, this is the time
     * to traverse x_i x_switch x_g.
     * 
     * there are three possible paths, of which one is sure to exist.
     * 
     * through x_switch
     * through x_limit
     * through x_mirror
     * 
     * this function returns the x_switch version.
     * 
     * TODO: return both t1 and t2
     * TODO: something about waiting time? i think this all might be wrong.
     */
    public static double tSwitch(double i, double idot, double g, double gdot, double umax) {
        // if (RRTStar7.goalRight(i, idot, g, gdot, umax)) {

        // these might return NaN for impossible paths

        double tIplusGminus = tSwitchIplusGminus(i, idot, g, gdot, umax);
        double tIminusGplus = tSwitchIminusGplus(i, idot, g, gdot, umax);

        // it's also possible for more than one path to be returned, one
        // faster than the other.

        if (Double.isNaN(tIplusGminus) || tIplusGminus > 1e100) {
            // there should be at least one solution
            if (Double.isNaN(tIminusGplus) || tIminusGplus > 1e100) {
                throw new IllegalArgumentException(String.format("A %f %f %f %f %f %f",
                        tIplusGminus, tIminusGplus, i, idot, g, gdot));
            }
            return tIminusGplus;

        }
        if (Double.isNaN(tIminusGplus) || tIminusGplus > 1e100) {
            return tIplusGminus;
        }
        // if we got here, then they're both actually numbers.
        // so just return the slower one for now.
        // TODO: be clever about waiting time etc.

        return Math.max(tIplusGminus, tIminusGplus);
    }

    /**
     * time to traverse x_i x_switch x_g.
     * 
     * This is for the I+G- path.
     */
    static double tSwitchIplusGminus(double i, double idot, double g, double gdot, double umax) {
        double q_dot_switch = qDotSwitchIplusGminus(i, idot, g, gdot, umax);
        // time from initial to switching point
        double t_1 = (q_dot_switch - idot) / umax;
        // time from switching to final, note i think this is a mistake
        // in the paper.
        double t_2 = (gdot - q_dot_switch) / (-1.0 * umax);
        return t_1 + t_2;
    }

    /**
     * Velocity at x_switch. this should be faster (more positive) than idot.
     * 
     * This is for the I+G- path.
     */
    static double qDotSwitchIplusGminus(double i, double idot, double g, double gdot, double umax) {
        return Math.sqrt(
                2 * umax * (qSwitchIplusGminus(i, idot, g, gdot, umax) - c_plus(i, idot, umax)));
    }

    /**
     * Position at x_switch, which is the midpoint between the curves
     * intersecting the initial and goal states.
     * 
     * This is for the I+G- path.
     */
    static double qSwitchIplusGminus(double i, double idot, double g, double gdot, double umax) {
        return (c_plus(i, idot, umax) + c_minus(g, gdot, umax)) / 2;
    }

    /**
     * Time to traverse x_i x_switch x_g.
     * 
     * This is for the I-G+ path.
     */
    static double tSwitchIminusGplus(double i, double idot, double g, double gdot, double umax) {
        double q_dot_switch = qDotSwitchIminusGplus(i, idot, g, gdot, umax);
        // time from initial to switching point
        double t_1 = (q_dot_switch - idot) / (-1.0 * umax);
        // time from switching to final, note i think this is a mistake
        // in the paper.
        double t_2 = (gdot - q_dot_switch) / umax;
        return t_1 + t_2;
    }

    /**
     * Velocity at x_switch. this should be faster (more negative) than idot.
     * 
     * This is for the I-G+ path.
     */
    static double qDotSwitchIminusGplus(double i, double idot, double g, double gdot, double umax) {
        return -1.0 * Math.sqrt(
                2 * umax * (qSwitchIminusGplus(i, idot, g, gdot, umax) - c_plus(g, gdot, umax)));
    }

    /**
     * Position at x_switch, which is the midpoint between the curves
     * intersecting the initial and goal states.
     * 
     * This is for the I-G+ path.
     */
    static double qSwitchIminusGplus(double i, double idot, double g, double gdot, double umax) {
        return (c_minus(i, idot, umax) + c_plus(g, gdot, umax)) / 2;
    }

    /** Intercept of negative-U path intersecting the state */
    static double c_minus(double x, double xdot, double umax) {
        return x - Math.pow(xdot, 2) / (-2.0 * umax);
    }

    /** Intercept of positive-U path intersecting the state */
    static double c_plus(double x, double xdot, double umax) {
        return x - Math.pow(xdot, 2) / (2.0 * umax);
    }

    static double tLimit() {
        return 0;
    }

    static double tMirror() {
        return 0;
    }

    /**
     * Used for the slower of the double-integrators; find a solution that takes the
     * specified time, to match the umax solution of the other double-integrator.
     * 
     * @param idot initial velocity
     * @param gdot goal velocity
     * @param tw   time to wait
     */
    public static double slowU(double i, double idot, double g, double gdot, double tw) {
        return Math.pow(gdot - idot, 2) / (2 * (g - i - i * tw));
        // return (gdot - idot) / tw;
    }

    /**
     * Return a list of nearby nodes, using the KDTree metric, which may not
     * actually contain the nearest nodes in non-Euclidean spaces. Returns the
     * single
     * nearest node if there are no other near nodes.
     */
    ArrayList<NearNode<N4>> Near(Matrix<N4, N1> x_new, KDNode<Node<N4>> rootNode) {
        ArrayList<NearNode<N4>> nearNodes = new ArrayList<>();
        KDTree.near(_model, rootNode, x_new, radius, (node, dist) -> {
            nearNodes.add(new NearNode<>(node, dist));
        });
        return nearNodes;
    }

    /**
     * Returns a member of X_near resulting in the lowest-cost path to x_new.
     * Removes infeasible nodes from X_near so we don't look at them again later.
     */
    Node<N4> ChooseParent(List<NearNode<N4>> X_near, Matrix<N4, N1> x_new) {
        Collections.sort(X_near);
        Iterator<NearNode<N4>> ni = X_near.iterator();
        while (ni.hasNext()) {
            NearNode<N4> nearNode = ni.next();
            ni.remove();
            if (CollisionFree(nearNode.node.getState(), x_new)) {
                return nearNode.node;
            }
        }
        return null;
    }

    /** Add the node link.target to the tree, with an edge from source to target. */
    Node<N4> InsertNode(LocalLink<N4> link, KDNode<Node<N4>> rootNode) {
        Graph.newLink(link.get_source(), link.get_target(), link.get_linkDist());
        KDTree.insert(_model, rootNode, link.get_target());
        return link.get_target();
    }

    /**
     * look through the nodes in X_near to see if any should be new children of
     * newNode.
     */
    void Rewire(List<NearNode<N4>> X_near, Node<N4> newNode, boolean timeForward) {
        if (DEBUG)
            System.out.printf("Rewire candidates %d\n", X_near.size());
        ListIterator<NearNode<N4>> li = X_near.listIterator(X_near.size());
        while (li.hasPrevious()) {
            NearNode<N4> jn = li.previous();
            if (jn.node.getIncoming() != null) {
                Matrix<N4, N1> x1 = newNode.getState();
                Matrix<N4, N1> x2 = jn.node.getState();
                // shortcut the inevitable duplicate
                if (x1.isEqual(x2, 0.01))
                    continue;
                if (DEBUG)
                    System.out.printf("Try rewiring %s to %s\n", Util.matStr(x1), Util.matStr(x2));
                ShootingSolver<N4, N2>.Solution sol = solver.solve(Nat.N4(), Nat.N2(), f, x1, x2, timeForward);
                if (sol == null) {
                    if (DEBUG)
                        System.out.println("no solution");
                } else {
                    if (Graph.rewire(_model, newNode, jn.node, Math.abs(sol.dt))) {
                        if (DEBUG)
                            System.out.println("REWIRED");
                    }
                }
            }
        }
    }

    @Override
    public List<Node<N4>> getNodesA() {
        ArrayList<Node<N4>> allNodes = new ArrayList<>();
        allNodes.addAll(KDTree.values(_T_a));
        return allNodes;
    }

    @Override
    public List<Node<N4>> getNodesB() {
        ArrayList<Node<N4>> allNodes = new ArrayList<>();
        allNodes.addAll(KDTree.values(_T_b));
        return allNodes;
    }

    @Override
    public Path<N4> getBestPath() {
        return _sigma_best;
    }

    /**
     * Starting from leaf node, walk the parent links to accumulate
     * the full path, and reverse it, to return a path from root to node.
     * 
     * populates list A only. bleah.
     * 
     * @param node leaf node
     */
    Path<N4> walkParents(Set<Node<N4>> visited, Node<N4> node) {
        // Collect the states along the path (backwards)
        List<Matrix<N4, N1>> configs = new LinkedList<>();
        // Since we're visiting all the nodes it's very cheap to verify the total
        // distance
        double totalDistance = 0;
        while (true) {
            if (visited.contains(node)) {
                System.out.println("found a cycle");
                throw new IllegalArgumentException();
            }
            visited.add(node);
            configs.add(node.getState());
            LinkInterface<N4> incoming = node.getIncoming();
            if (incoming == null)
                break;
            totalDistance += incoming.get_linkDist();
            node = incoming.get_source();
        }
        // now we have the forwards list of states
        Collections.reverse(configs);

        return new Path<>(totalDistance, configs, new LinkedList<>());
    }

    @Override
    public void setStepNo(int stepNo) {
        if (stepNo < 1)
            throw new IllegalArgumentException();
        this.stepNo = stepNo;
        this.radius = _gamma * Math.pow(Math.log(stepNo + 1) / (stepNo + 1), 0.25);
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    static boolean same(Matrix<N4, N1> a, Matrix<N4, N1> b) {
        return a.isEqual(b, 0.0001);
    }
}