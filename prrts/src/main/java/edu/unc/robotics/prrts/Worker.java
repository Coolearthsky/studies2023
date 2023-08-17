package edu.unc.robotics.prrts;

import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import edu.unc.robotics.prrts.kdtree.KDModel;
import edu.unc.robotics.prrts.kdtree.KDNearCallback;
import edu.unc.robotics.prrts.kdtree.KDTraversal;
import edu.unc.robotics.prrts.tree.Link;
import edu.unc.robotics.prrts.tree.NearNode;
import edu.unc.robotics.prrts.tree.Node;
import edu.unc.robotics.prrts.util.MersenneTwister;

class Worker implements Runnable, KDNearCallback<Node> {
    private static final int INITIAL_NEAR_LIST_CAPACITY = 1024;

    private final KDModel _kdModel;
    private final int _dimensions;
    private final int _workerNo;
    private final int _threadCount;
    private final KDTraversal<Node> _kdTraversal;
    private final RobotModel _robotModel;
    private final double[] _sampleMin;
    private final double[] _sampleMax;
    private final Random _random;
    private final double _gamma;
    private final long _timeLimit;
    private final long _startTime;
    private final int _sampleLimit;
    private final CountDownLatch _doneLatch;
    private final AtomicInteger _stepNo;
    private final AtomicReference<Link> _bestPath;
    private final AtomicBoolean _done;

    private NearNode[] _nearList = new NearNode[INITIAL_NEAR_LIST_CAPACITY];

    public Worker(
            KDModel kdModel,
            KDTraversal<Node> kdTraversal,
            RobotModel robotModel,
            double gamma,
            int workerNo,
            int threadCount,
            long timeLimit,
            long startTime,
            int sampleLimit,
            CountDownLatch doneLatch,
            AtomicInteger stepNo,
            AtomicReference<Link> bestPath,
            AtomicBoolean done) {
        _kdModel = kdModel;
        _kdTraversal = kdTraversal;
        _robotModel = robotModel;
        _gamma = gamma;
        _dimensions = _kdModel.dimensions();
        _workerNo = workerNo;
        _threadCount = threadCount;
        _timeLimit = timeLimit;
        _startTime = startTime;
        _sampleLimit = sampleLimit;
        _random = new MersenneTwister(workerNo);
        _sampleMin = new double[_kdModel.dimensions()];
        _sampleMax = new double[_kdModel.dimensions()];
        _doneLatch = doneLatch;
        _stepNo = stepNo;
        _bestPath = bestPath;
        _done = done;
    }

    /**
     * Callback handler for calls to kdNear. This method adds near nodes
     * to the worker's nearList.
     *
     * @param target
     * @param index
     * @param config this does nothing
     * @param value
     * @param dist
     */
    @Override
    public void kdNear(double[] target, int index, double[] config, Node value, double dist) {
        if (index == _nearList.length) {
            _nearList = Arrays.copyOf(_nearList, index * 2);
        }

        _nearList[index] = new NearNode(value.get_link().get(), dist);
        // _nearList[index] = new NearNode(value, dist);

        // NearNode n = _nearList[index];

        // if (n == null) {
        // _nearList[index] = n = new NearNode();
        // }

        // n.link = value.get_link().get();
        // n.linkDist = dist;
        // n._pathDist = n.link.get_pathDist() + dist;
    }

    /**
     * write random doubles into config
     * 
     * @param config OUTVAR result
     */
    private void randomize(double[] config) {
        for (int i = _dimensions; --i >= 0;) {
            config[i] = _random.nextDouble() * (_sampleMax[i] - _sampleMin[i])
                    + _sampleMin[i];
        }
    }

    /**
     * @return true if a new sample was added.
     */
    private boolean step(int stepNo, double[] newConfig) {
        // generate a new random sample
        randomize(newConfig);

        if (!_robotModel.clear(newConfig)) {
            return false;
        }

        double radius = _gamma * Math.pow(
                Math.log(stepNo + 1) / (stepNo + 1),
                1.0 / _dimensions);

        int nearCount = _kdTraversal.near(newConfig, radius, this);

        if (nearCount == 0) {
            // near() found nothing nearby

            Node nearest = _kdTraversal.nearest(newConfig);
            double distToNearest = _kdTraversal.distToLastNearest();

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
                    nearest.get_link().get());

            Operations.updateBestPath(_bestPath, newNode.get_link().get());

            _kdTraversal.insert(newConfig, newNode);
            return true;
        }

        // Sort the array from nearest to farthest. After sorting
        // we can traverse the array sequentially and select the first
        // configuration that can link. We know that anything after it
        // in the array will be further away, and thus potentially save
        // a lot of calls to the (usually) expensive link() method.
        Arrays.sort(_nearList, 0, nearCount);

        for (int i = 0; i < nearCount; ++i) {
            Link link = _nearList[i].link;

            if (!_robotModel.link(link.get_node().get_config(), newConfig)) {
                // help GC
                // _nearList[i].link = null;
                continue;
            }

            // Found a linkable configuration. Create the node
            // and link it in here.

            Node newNode = new Node(
                    newConfig,
                    _robotModel.goal(newConfig),
                    _nearList[i].linkDist,
                    link);

            Operations.updateBestPath(_bestPath, newNode.get_link().get());

            // Put the node in the KD-Tree. After insertion,
            // other threads will "see" the new node and may start
            // rewiring it.

            _kdTraversal.insert(newConfig, newNode);

            // help GC
            // _nearList[i].link = null;

            // For the remaining nodes in the near list, rewire
            // their links to go through the newly inserted node
            // if doing so is feasible and would shorten their path
            //
            // We go through the remaining list in reverse order to
            // reduce the number of rewirings we do on the farther nodes.
            // If we went from nearest to farthest, the far nodes might
            // rewire through the near nodes, then through the newly added
            // node.
            for (int j = nearCount; --j > i;) {
                Operations.rewire(_bestPath, _robotModel, _nearList[j].link, _nearList[j].linkDist, newNode, radius,
                        _threadCount);

                // help GC
                // _nearList[j].link = null;
            }

            return true;
        }

        // if we're here, we've looped through the entire near list and
        // found no nodes that can be linked through. We return false
        // indicating we failed to add a node.
        return false;
    }

    private void generateSamples() {
        // only have 1 worker check the time limit
        final boolean checkTimeLimit = (_workerNo == 0) && (_timeLimit > 0);
        double[] newConfig = new double[_dimensions];
        int stepNo = _stepNo.get();

        while (!_done.get()) {
            if (step(stepNo, newConfig)) {
                stepNo = _stepNo.incrementAndGet();
                if (stepNo > _sampleLimit) {
                    _done.set(true);
                }
                // sample was added, create a new one
                newConfig = new double[_dimensions];
            } else {
                // failed add a sample, refresh the step no
                // and try again
                stepNo = _stepNo.get();
            }

            if (checkTimeLimit) {
                long now = System.nanoTime();
                if (now - _startTime > _timeLimit) {
                    _done.set(true);
                }
            }
        }
    }

    public void run() {
        try {
            _kdModel.getBounds(_sampleMin, _sampleMax);
            generateSamples();
        } finally {
            _doneLatch.countDown();
        }
    }
}