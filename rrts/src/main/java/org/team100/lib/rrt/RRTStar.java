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

public class RRTStar<T extends KDModel & RobotModel> implements Solver {
    private final T _model;
    private final KDNode<Node> _rootNode;
    private final Sample _sample;
    private final double _gamma;
    private LinkInterface _bestPath;

    // mutable loop variables to make the loop code cleaner
    int stepNo = 0;
    double[] x_rand;

    public RRTStar(T model, Sample sample, double gamma) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        _model = model;
        _rootNode = new KDNode<Node>(new Node(model.initial()));
        _sample = sample;
        _gamma = gamma;
        _bestPath = null;
    }

    double[] SampleFree() {
        double[] newConfig = _sample.get();
        if (!_model.clear(newConfig))
            return null;
        return newConfig;
    }

    @Override
    public void setStepNo(int stepNo) {
        if (stepNo < 1)
            throw new IllegalArgumentException();
        this.stepNo = stepNo;
    }

    /**
     * @return true if a new sample was added.
     */
    @Override
    public int step() {
        x_rand = SampleFree();
        if (x_rand == null)
            return 0;

        double radius = _gamma * Math.pow(
                Math.log(stepNo + 1) / (stepNo + 1),
                1.0 / _model.dimensions());

        List<NearNode> nearNodes = new ArrayList<>();
        KDTree.near(_model, _rootNode, x_rand, radius, (node, dist) -> {
            if (node.getIncoming() != null)
                nearNodes.add(new NearNode(node, dist));
        });

        if (nearNodes.isEmpty()) {

            KDNearNode<Node> nearResult = KDTree.nearest(_model, _rootNode, x_rand);
            Node nearest = nearResult._nearest;
            double distToNearest = nearResult._dist;

            if (distToNearest > radius) {
                _model.setStepNo(stepNo);
                _model.setRadius(radius);
                x_rand = _model.steer(nearResult, x_rand);
            }

            if (!_model.clear(x_rand)) {
                return 0;
            }

            if (!_model.link(nearest.getState(), x_rand)) {
                return 0;
            }

            // the new node has the new sampled config, the distance(cost) to the
            // nearest other node we found above, and the "parent" is the "link"
            // from that nearest node.
            Node newTarget = new Node(x_rand);
            // recalculate dist just to be safe.
            LinkInterface newLink = Graph.newLink(_model, nearest, newTarget);

            _bestPath = Graph.chooseBestPath(_model, _bestPath, newLink);

            KDTree.insert(_model, _rootNode, newTarget);
            return 1;
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
            LinkInterface newLink = Graph.newLink(nearNode.node, newNode, nearNode.linkDist);
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
            return 1;
        }
        // no feasible link possible.
        return 0;
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
}