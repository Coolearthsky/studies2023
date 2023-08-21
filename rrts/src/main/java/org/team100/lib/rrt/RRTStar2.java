package org.team100.lib.rrt;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

import org.team100.lib.graph.Graph;
import org.team100.lib.graph.Link;
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
 * There are many similar-but-not-identical variations; I've kind of mashed them together.
 * 
 * x_rand <- Sample() // start with a random point
 * x_nearest <- Nearest(x_rand) // find the nearest node in the tree to x_rand
 * x_new <- Steer(x_nearest, x_rand) // find a point feasible from x_nearest
 * 
 * 
 * References:
 * 
 * http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf
 * https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/SamplingBasedMotionPlanning.pdf
 * http://lavalle.pl/planning/book.pdf
 * https://dspace.mit.edu/handle/1721.1/78449 
 * https://dspace.mit.edu/handle/1721.1/79884
 * https://natanaso.github.io/ece276b2018/ref/ECE276B_8_SamplingBasedPlanning.pdf
 * 
 */
public class RRTStar2<T extends KDModel & RobotModel> implements Solver {
    private final T _model;
    private final KDNode<Node> _rootNode;
    private final Sample _sample;
    private final double _gamma;
    private Link _bestPath;

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
     * @return true if a new sample was added.
     */
    @Override
    public boolean step(int stepNo) {
        // start with a random point

        double[] x_rand = SampleFree();
        if (x_rand == null)
            return false;
        
        // find the nearest node in the tree to x_rand

        KDNearNode<Node> x_nearest = KDTree.nearest(_model, _rootNode, x_rand);

        double[] x_new = _model.steer(stepNo, x_nearest, x_rand);



        double radius = _gamma * Math.pow(
                Math.log(stepNo + 1) / (stepNo + 1),
                1.0 / _model.dimensions());

        List<NearNode> nearNodes = new ArrayList<>();
        KDTree.near(_model, _rootNode, x_rand, radius, (node, dist) -> {
            if (node.getIncoming() != null)
                nearNodes.add(new NearNode(node, dist));
        });

        if (nearNodes.isEmpty()) {

            if (x_nearest._dist > radius) {
                x_rand = _model.steer(stepNo, x_nearest, x_rand);
            }

            if (!_model.clear(x_rand)) {
                return false;
            }

            if (!_model.link(x_nearest._nearest.getState(), x_rand)) {
                return false;
            }

            // the new node has the new sampled config, the distance(cost) to the
            // nearest other node we found above, and the "parent" is the "link"
            // from that nearest node.
            Node newTarget = new Node(x_rand);

            // recalculate dist just to be safe.
            Link newLink = Graph.newLink(_model, x_nearest._nearest, newTarget);

            _bestPath = Graph.chooseBestPath(_model, _bestPath, newLink);

            KDTree.insert(_model, _rootNode, newTarget);
            return true;
        }

        // Sort the array by total distance (including the distance to the new node).
        // We take the best (shortest) feasible node.
        Collections.sort(nearNodes);

        Iterator<NearNode> ni = nearNodes.iterator();
        while (ni.hasNext()) {
            NearNode nearNode = ni.next();
            ni.remove();

            if (!_model.link(nearNode.node.getState(), x_rand)) {
                continue;
            }

            // Found a linkable configuration.
            Node newNode = new Node(x_rand);
            Link newLink = Graph.newLink(nearNode.node, newNode, nearNode.linkDist);
            _bestPath = Graph.chooseBestPath(_model, _bestPath, newLink);
            KDTree.insert(_model, _rootNode, newNode);

            // check the remaining nearby nodes to see if they would be better
            // as children of the new node
            ListIterator<NearNode> li = nearNodes.listIterator(nearNodes.size());
            while (li.hasPrevious()) {
                NearNode jn = li.previous();
                if (Graph.rewire(_model, newNode, jn.node, jn.linkDist)) {
                    _bestPath = Graph.chooseBestPath(_model, _bestPath, newNode.getIncoming());
                }
            }
            return true;
        }
        // no feasible link possible.
        return false;
    }

    double[] SampleFree() {
        double[] newConfig = _sample.get();
        if (!_model.clear(newConfig))
            return null;
        return newConfig;
    }

    @Override
    public Iterable<Node> getNodes() {
        return KDTree.values(_rootNode);
    }

    @Override
    public Path getBestPath() {
        Link link = _bestPath;
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
            Link incoming = node.getIncoming();
            if (incoming == null)
                break;
            totalDistance += incoming.get_linkDist();
            node = incoming.get_source();
        }
        // now we have the forwards list of states
        Collections.reverse(configs);

        return new Path(totalDistance, configs);
    }
}