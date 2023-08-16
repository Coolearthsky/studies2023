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

    void updateBestPath(Link link, double radius) {
        Link currentBestPath;
        double distToGoal;
        Node node = link.get_node();

        if (node.is_inGoal()) {
            distToGoal = link.get_pathDist();
        } else {
            return;
        }

        do {
            currentBestPath = _bestPath.get();

            if (currentBestPath != null) {
                double bestDist = currentBestPath.get_pathDist();
                if (distToGoal >= bestDist) {
                    return;
                }
            }
        } while (!_bestPath.compareAndSet(currentBestPath, link));
    }

    private void updateChildren(Link newParent, Link oldParent, double radius) {
        assert newParent.get_node() == oldParent.get_node() : "updating links of different nodes";
        assert oldParent.isExpired() : "updating non-expired link";
        assert newParent.get_pathDist() <= oldParent.get_pathDist() : "updating to longer path";

        for (;;) {
            Link oldChild = oldParent.removeFirstChild();

            if (oldChild == null) {
                // done.

                if (newParent.isExpired()) {
                    oldParent = newParent;
                    newParent = oldParent.get_node().get_link().get();
                    continue;
                }

                return;
            }

            if (oldChild.isExpired()) {
                assert _threadCount > 1;
                continue;
            }

            Node node = oldChild.get_node();

            if (node.get_link().get().get_parent().get_node() != oldParent.get_node()) {
                continue;
            }

            Link newChild = node.setLink(oldChild, oldChild.get_linkDist(), newParent);

            if (newChild != null) {
                updateChildren(newChild, oldChild, radius);
                updateBestPath(newChild, radius);
            } else {
                assert _threadCount > 1;
                assert node.get_link().get() != oldChild;
            }
        }
    }

    private void rewire(Link oldLink, double linkDist, Node newParent, double radius) {
        assert oldLink.get_parent() != null;
        assert oldLink.get_parent().get_node() != newParent;

        Node node = oldLink.get_node();

        Link parentLink = newParent.get_link().get();

        double pathDist = parentLink.get_pathDist() + linkDist;

        // check if rewiring would create a shorter path
        if (pathDist >= oldLink.get_pathDist()) {
            return;
        }

        // check if rewiring is possible
        if (!_robotModel.link(oldLink.get_node().get_config(), newParent.get_config())) {
            return;
        }

        // rewire the node. this loop continues to attempt atomic
        // updates until either the update succeeds or the pathDist
        // of the oldLink is found to be better than what we're trying
        // to put in
        do {
            Link newLink = node.setLink(oldLink, linkDist, parentLink);

            if (newLink != null) {
                updateChildren(newLink, oldLink, radius);
                updateBestPath(newLink, radius);

                if (parentLink.isExpired()) {
                    updateChildren(parentLink.get_node().get_link().get(), parentLink, radius);
                }

                // Setting newLink expires oldLink but doesn not remove
                // it from its parent. Here we do a little cleanup.
                // We do it after the expired parent check since the parent
                // will likely have already cleaned this up, and this call
                // will be O(1) instead of O(n)
                if (!oldLink.get_parent().removeChild(oldLink)) {
                    assert _threadCount > 1 : "concurrent update running with 1 thread";
                }

                return;
            }

            assert _threadCount > 1 : "concurrent update running with 1 thread";

            Link updatedOldLink = node.get_link().get();

            assert updatedOldLink != oldLink;

            oldLink = updatedOldLink;

        } while (pathDist < oldLink.get_pathDist());
    }

    /**
     * Callback handler for calls to kdNear. This method adds near nodes
     * to the worker's nearList.
     *
     * @param target
     * @param index
     * @param config
     * @param value
     * @param dist
     */
    @Override
    public void kdNear(double[] target, int index, double[] config, Node value, double dist) {
        if (index == _nearList.length) {
            _nearList = Arrays.copyOf(_nearList, index * 2);
        }

        NearNode n = _nearList[index];

        if (n == null) {
            _nearList[index] = n = new NearNode();
        }

        n.link = value.get_link().get();
        n.linkDist = dist;
        n._pathDist = n.link.get_pathDist() + dist;
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
            // nothing within radius
            Node nearest = _kdTraversal.nearest(newConfig);
            double distToNearest = _kdTraversal.distToLastNearest();

            assert radius < distToNearest;

            //steer(newConfig, nearest.get_config(), radius / distToNearest);
            _kdModel.steer(nearest.get_config(),newConfig, radius / distToNearest);

            if (!_robotModel.clear(newConfig)) {
                return false;
            }

            if (!_robotModel.link(nearest.get_config(), newConfig)) {
                return false;
            }

            // This should be radius, but might be off slightly so we
            // recalculate just to be safe.
            distToNearest = _kdModel.dist(newConfig, nearest.get_config());

            Node newNode = new Node(
                    newConfig, _robotModel.goal(newConfig), distToNearest, nearest.get_link().get());

            updateBestPath(newNode.get_link().get(), radius);

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
                _nearList[i].link = null;
                continue;
            }

            // Found a linkable configuration. Create the node
            // and link it in here.

            Node newNode = new Node(
                    newConfig, _robotModel.goal(newConfig), _nearList[i].linkDist, link);

            updateBestPath(newNode.get_link().get(), radius);

            // Put the node in the KD-Tree. After insertion,
            // other threads will "see" the new node and may start
            // rewiring it.

            _kdTraversal.insert(newConfig, newNode);

            // help GC
            _nearList[i].link = null;

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
                rewire(_nearList[j].link, _nearList[j].linkDist, newNode, radius);

                // help GC
                _nearList[j].link = null;
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