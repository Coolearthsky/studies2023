package org.team100.lib.rrt;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

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
 * This is an attempt to make the code below look more like the pseudocode.
 * 
 * There are many similar-but-not-identical variations; I've kind of mashed them
 * together.
 * 
 * x_rand <- Sample() // start with a random point
 * x_nearest <- Nearest(x_rand) // find the nearest node in the tree to x_rand
 * x_new <- Steer(x_nearest, x_rand) // find a point feasible from x_nearest
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
 */
public class RRTStar2<T extends KDModel & RobotModel> implements Solver {
    private final T _model;
    private final KDNode<Node> _rootNode;
    private final Sample _sample;
    private final double _gamma;
    private LinkInterface _bestPath;

    // mutable loop variables to make the loop code cleaner
    int stepNo;
    double radius;
    double[] x_rand;
    KDNearNode<Node> x_nearest;
    double[] x_new;
    boolean single_nearest;

    public RRTStar2(T model, Sample sample, double gamma) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        _model = model;
        _rootNode = new KDNode<Node>(new Node(model.initial()));
        _sample = sample;
        _gamma = gamma;
        _bestPath = null;
    }

    /**
     * @param stepNo must be greater than zero
     * @return true if a new sample was added.
     */
    @Override
    public int step() {
        // start with a random point
        x_rand = SampleFree();

        // find the nearest node in the tree to x_rand
        x_nearest = Nearest(x_rand);

        // find a feasible point somewhere nearby
        x_new = Steer(x_nearest, x_rand);

        if (CollisionFree(x_nearest._nearest.getState(), x_new)) {

            // make a list of points near the feasible one
            List<NearNode> X_near = Near(x_new);

            // Sort the array by total distance (including the distance to the new node).
            // We take the best (shortest) feasible node.
            Collections.sort(X_near);

            Iterator<NearNode> ni = X_near.iterator();
            while (ni.hasNext()) {
                NearNode nearNode = ni.next();
                ni.remove();

                if (!_model.link(nearNode.node.getState(), x_rand)) {
                    // System.out.println("no link");
                    continue;
                }

                // Found a linkable configuration.
                Node newNode = new Node(x_rand);
                LinkInterface newLink = Graph.newLink(_model, nearNode.node, newNode);
                _bestPath = Graph.chooseBestPath(_model, _bestPath, newLink);
                KDTree.insert(_model, _rootNode, newNode);

                // check the remaining nearby nodes to see if they would be better
                // as children of the new node
                ListIterator<NearNode> li = X_near.listIterator(X_near.size());
                while (li.hasPrevious()) {
                    NearNode jn = li.previous();
                    if (Graph.rewire(_model, newNode, jn.node, jn.linkDist)) {
                        _bestPath = Graph.chooseBestPath(_model, _bestPath, newNode.getIncoming());
                    }
                }
                return 1;
            }
            // no feasible link possible.
        }
        return 0;
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
     */
    KDNearNode<Node> Nearest(double[] x_rand) {
        return KDTree.nearest(_model, _rootNode, x_rand);
    }

    /**
     * Return a state that tries to go from x_nearest to x_rand using a feasible
     * trajectory. For simple systems "feasible" just means "closer."
     * 
     * @param x_nearest containts distance to x_rand
     */
    double[] Steer(KDNearNode<Node> x_nearest, double[] x_rand) {
        if (x_nearest._dist < radius) {
            single_nearest = false;
            return x_rand;
        }
        single_nearest = true;
        _model.setStepNo(stepNo);
        _model.setRadius(radius);
        double[] x_new = _model.steer(x_nearest, x_rand);
        return x_new;
    }

    /**
     * Return a list of nearby nodes, using the KDTree metric, which may not
     * actually contain the nearest nodes in non-Euclidean spaces. This function and
     * the Steer function should "match" i.e. the "Near" function should always
     * return the "seed" of the "Steer" function.
     */
    List<NearNode> Near(double[] x_new) {
        List<NearNode> nearNodes = new ArrayList<>();
        // if there's just one nearest node, return it alone
        if (single_nearest) {
            nearNodes.add(new NearNode(x_nearest._nearest, x_nearest._dist));
            return nearNodes;
        }

        KDTree.near(_model, _rootNode, x_new, radius, (node, dist) -> {
            nearNodes.add(new NearNode(node, dist));
        });
        // this should really never happen
        if (nearNodes.isEmpty())
            nearNodes.add(new NearNode(x_nearest._nearest, x_nearest._dist));

        return nearNodes;
    }

    @Override
    public List<Node> getNodesA() {
        return KDTree.values(_rootNode);
    }

    @Override
    public List<Node> getNodesB() {
        return new ArrayList<Node>();
    }

    @Override
    public Path getBestPath() {
        LinkInterface link = _bestPath;
        if (link == null) {
            return null;
        }
        Node node = link.get_target();

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

        return new Path(totalDistance, configs,new LinkedList<double[]>());
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