package edu.unc.robotics.prrts;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

import edu.unc.robotics.prrts.kdtree.KDModel;
import edu.unc.robotics.prrts.kdtree.KDNearNode;
import edu.unc.robotics.prrts.kdtree.KDNode;
import edu.unc.robotics.prrts.kdtree.Util;
import edu.unc.robotics.prrts.tree.Link;
import edu.unc.robotics.prrts.tree.NearNode;
import edu.unc.robotics.prrts.tree.Node;

class Worker {
    private final KDModel _kdModel;
    private final KDNode<Node> _rootNode ;
    private final RobotModel _robotModel;
    private final Sample _sample;
    private final double _gamma;
    private final long _timeLimit;
    private final long _startTime;
    private final int _sampleLimit;
    private  int _stepNo;
    public Link _bestPath;

    public Worker(
            KDModel kdModel,
            KDNode<Node> rootNode ,
            RobotModel robotModel,
            Sample sample,
            double gamma,
            long timeLimit,
            long startTime,
            int sampleLimit) {
        _kdModel = kdModel;
        _rootNode = rootNode;
        _robotModel = robotModel;
        _sample = sample;
        _gamma = gamma;
        _timeLimit = timeLimit;
        _startTime = startTime;
        _sampleLimit = sampleLimit;
        _stepNo = 0;
        _bestPath = null;
    }
    
    public int getStepNo() {
        return _stepNo;
    }

    /**
     * @return true if a new sample was added.
     */
    private boolean step(int stepNo) {

        double[] newConfig = _sample.get();

        // this is wrong, we want to check the steered sample.
        if (!_robotModel.clear(newConfig)) {
            return false;
        }

        double radius = _gamma * Math.pow(
                Math.log(stepNo + 1) / (stepNo + 1),
                1.0 / _kdModel.dimensions());

        List<NearNode> nearNodes = new ArrayList<>();
        Util.near(_kdModel, _rootNode, newConfig, radius, (v, d) -> {
            nearNodes.add(new NearNode(v.get_link(), d));
        });

        if (nearNodes.isEmpty()) {

            KDNearNode<Node> nearResult = Util.nearest(_kdModel, _rootNode, newConfig);
            Node nearest = nearResult._nearest;
            double distToNearest =  nearResult._dist;

            if (distToNearest > radius) {
                // usually the "nearest" node is outside the radius, so bring it closer
                _kdModel.steer(nearest.get_config(), newConfig, radius / distToNearest);
            }

            if (!_robotModel.clear(newConfig)) {
                return false;
            }

            if (!_robotModel.link(nearest.get_config(), newConfig)) {
                return false;
            }

            // This should be radius, but might be off slightly so we
            // recalculate just to be safe.
            distToNearest = _kdModel.dist(newConfig, nearest.get_config());

            // the new node has the new sampled config, the distance(cost) to the
            // nearest other node we found above, and the "parent" is the "link"
            // from that nearest node.
            Node newNode = new Node(
                    newConfig,
                    _robotModel.goal(newConfig),
                    distToNearest,
                    nearest.get_link());

            _bestPath = Operations.updateBestPath(_bestPath, newNode.get_link());

            Util.insert(_kdModel, _rootNode, newConfig, newNode);
            return true;
        }

        // Sort the array from nearest to farthest. After sorting
        // we can traverse the array sequentially and select the first
        // configuration that can link. We know that anything after it
        // in the array will be further away, and thus potentially save
        // a lot of calls to the (usually) expensive link() method.
        Collections.sort(nearNodes);

        Iterator<NearNode> ni = nearNodes.iterator();
        while (ni.hasNext()) {
            NearNode nn = ni.next();
            ni.remove();
            Link link = nn.link;

            if (!_robotModel.link(link.get_node().get_config(), newConfig)) {
                continue;
            }

            // Found a linkable configuration. Create the node
            // and link it in here.

            Node newNode = new Node(
                    newConfig,
                    _robotModel.goal(newConfig),
                    nn.linkDist,
                    link);

           _bestPath = Operations.updateBestPath(_bestPath, newNode.get_link());

            Util.insert(_kdModel, _rootNode, newConfig, newNode);

            // For the remaining nodes in the near list, rewire
            // their links to go through the newly inserted node
            // if doing so is feasible and would shorten their path
            //
            // We go through the remaining list in reverse order to
            // reduce the number of rewirings we do on the farther nodes.
            // If we went from nearest to farthest, the far nodes might
            // rewire through the near nodes, then through the newly added
            // node.

            ListIterator<NearNode> li = nearNodes.listIterator(nearNodes.size());
            while (li.hasPrevious()) {
                NearNode jn = li.previous();

                // rewiring needs to be informed by the dynamics; turn it off for now
               _bestPath = Operations.rewire(_bestPath, _robotModel, jn.link, jn.linkDist, newNode);
            }

            return true;
        }

        // if we're here, we've looped through the entire near list and
        // found no nodes that can be linked through. We return false
        // indicating we failed to add a node.
        return false;
    }

    public void run() {        
        while (true) {
            if (step(_stepNo)) {
                _stepNo++;
                if (_stepNo > _sampleLimit) {
                    return;
                }
            }
        
            if (_timeLimit > 0) {
                long now = System.nanoTime();
                if (now - _startTime > _timeLimit) {
                    return;
                }
            }
        }
    }
}