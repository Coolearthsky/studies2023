package edu.unc.robotics.prrts;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.unc.robotics.prrts.kdtree.KDModel;
import edu.unc.robotics.prrts.kdtree.KDNearCallback;
import edu.unc.robotics.prrts.kdtree.KDTraversal;
import edu.unc.robotics.prrts.kdtree.KDTree;
import edu.unc.robotics.prrts.tree.Link;
import edu.unc.robotics.prrts.tree.NearNode;
import edu.unc.robotics.prrts.tree.Node;
import edu.unc.robotics.prrts.util.MersenneTwister;

/**
 * PRRTStar
 *
 * @author jeffi
 */
public class PRRTStar {
    private static final Logger _log = Logger.getLogger(PRRTStar.class.getName());

    private static final int INITIAL_NEAR_LIST_CAPACITY = 1024;

    // Robotic System
    KDModel _kdModel;
    Supplier<RobotModel> _robotModelProvider;
    Supplier<Random> _randomProvider = () -> new MersenneTwister();

    // RRT* Parameters
    double[] _startConfig;
    double _gamma = 5.0;
    boolean _perThreadRegionSampling = true;
    int _regionSplitAxis = 0;
    int _samplesPerStep = 1;
    double[] _targetConfig;

    // Run duration
    int _threadCount;
    long _timeLimit;
    long _startTime;
    int _sampleLimit;

    // Runtime data
    KDTree<Node> _kdTree;
    final AtomicInteger _stepNo = new AtomicInteger(0);
    final AtomicBoolean _done = new AtomicBoolean(false);
    CountDownLatch _doneLatch;
    final AtomicReference<Link> _bestPath = new AtomicReference<Link>();

    public PRRTStar(KDModel kdModel, Supplier<RobotModel> robotModelProvider, double[] init) {
        _kdModel = kdModel;
        _robotModelProvider = robotModelProvider;
        _startConfig = init;
        _kdTree = new KDTree<Node>(kdModel, init, new Node(init, false));
    }

    /**
     * Sets the gamma value (aka step size) for RRT*. The default value is 5.
     *
     * @param gamma the value to set.
     */
    public void setGamma(double gamma) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        _gamma = gamma;
    }

    /**
     * Enables per-thread region based sampling. The default value is true.
     *
     * @param b
     */
    public void setPerThreadRegionSampling(boolean b) {
        _perThreadRegionSampling = b;
    }

    /**
     * Sets a pseudo-random number generator provider. The provider is asked
     * to provide random number generators for each of the threads at runtime.
     *
     * @param randomProvider
     */
    public void setRandomProvider(Supplier<Random> randomProvider) {
        _randomProvider = randomProvider;
    }

    /**
     * Returns the current step no. May be called while running, in which
     * case the value returned will be less than or equal to the actual
     * step no. Called after running, this may return more than the requested
     * number of samples since multiple threads may finish concurrently.
     *
     * @return the approximate step number.
     */
    public int getStepNo() {
        return _stepNo.get();
    }

    public Iterable<Node> getNodes() {
        return _kdTree.values();
    }

    /**
     * Returns the best path found so far. This method is safe to be called
     * while PRRT* is running.
     *
     * @return the best path, or null if none found so far.
     */
    public Path getBestPath() {
        Link link = _bestPath.get();
        if (link == null) {
            return null;
        }
        List<double[]> configs = new LinkedList<double[]>();
        double pathDist = link.pathDist;
        if (!link.node.inGoal) {
            assert _targetConfig != null;
            configs.add(_targetConfig);
            pathDist += _kdModel.dist(link.node.config, _targetConfig);
        }
        for (; link != null; link = link.parent.get()) {
            configs.add(link.node.config);
        }
        Collections.reverse(configs);
        return new Path(pathDist, configs);
    }

    public Path runForDuration(int threadCount, long milliseconds) {
        return runForDuration(threadCount, milliseconds, TimeUnit.MILLISECONDS);
    }

    public Path runForDuration(int threadCount, long duration, TimeUnit timeUnit) {
        if (duration <= 0) {
            throw new IllegalArgumentException("invalid duration, must be > 0");
        }
        return run(threadCount, Integer.MAX_VALUE, duration, timeUnit);
    }

    public Path runSamples(int threadCount, int samples) {
        return run(threadCount, samples, 0, null);
    }

    public Path runIndefinitely(int threadCount) {
        return run(threadCount, Integer.MAX_VALUE, 0, null);
    }

    private Path run(int threadCount, int sampleLimit, long duration, TimeUnit timeUnit) {
        if (threadCount < 1) {
            throw new IllegalArgumentException("thread count must be >= 1");
        }

        int availableProcessors = Runtime.getRuntime().availableProcessors();

        if (threadCount > availableProcessors) {
            _log.warning(String.format(
                    "Thread count (%d) exceeds available processors (%d)",
                    threadCount, availableProcessors));
        }

        _doneLatch = new CountDownLatch(threadCount);
        _sampleLimit = sampleLimit;
        _timeLimit = duration > 0 ? timeUnit.toNanos(duration) : 0;

        _startTime = System.nanoTime();

        Worker[] workers = new Worker[threadCount];
        for (int i = 0; i < threadCount; ++i) {
            workers[i] = new Worker(i, threadCount);
        }

        ThreadGroup threadGroup = Thread.currentThread().getThreadGroup();
        Thread[] threads = new Thread[threadCount];
        for (int i = 1; i < threadCount; ++i) {
            threads[i] = new Thread(threadGroup, workers[i]);
            threads[i].start();
        }

        // worker 0 runs on the calling thread (thus if only 1 thread is
        // specified, no additional threads are created)
        workers[0].run();

        // We could join all worker threads here, but a latch will allow
        // the calling thread to continue sooner.
        try {
            if (!_doneLatch.await(1, TimeUnit.SECONDS)) {
                _log.warning("waiting too long for workers, trying to join");
                for (int i = 1; i < threadCount; ++i) {
                    threads[i].join();
                }
            }
        } catch (InterruptedException e) {
            _log.log(Level.WARNING, "Interrupted", e);
        }

        return getBestPath();
    }

    class Worker implements Runnable, KDNearCallback<Node> {
        final int _dimensions = _kdModel.dimensions();
        final int _workerNo;
        final int _threadCount;

        KDTraversal<Node> _kdTraversal;
        RobotModel _robotModel;
        double[] _sampleMin;
        double[] _sampleMax;
        Random _random;

        NearNode[] _nearList = new NearNode[INITIAL_NEAR_LIST_CAPACITY];

        public Worker(int workerNo, int threadCount) {
            _workerNo = workerNo;
            _threadCount = threadCount;
        }

        void updateBestPath(Link link, double radius) {
            Link currentBestPath;
            double distToGoal;
            Node node = link.node;

            if (node.inGoal) {
                distToGoal = link.pathDist;
            } else if (_targetConfig != null) {
                // searching for a configuration that can reach a pre-specified
                // target configuration

                double distToTarget = _kdModel.dist(node.config, _targetConfig);
                if (distToTarget > radius ||
                        !_robotModel.link(node.config, _targetConfig)) {
                    return;
                }
                distToGoal = link.pathDist + distToTarget;
            } else {
                return;
            }

            do {
                currentBestPath = _bestPath.get();

                if (currentBestPath != null) {
                    double bestDist = currentBestPath.pathDist;

                    if (!currentBestPath.node.inGoal) {
                        // current best is not a goal itself, so it must
                        // link to a target configuration
                        bestDist += _kdModel.dist(currentBestPath.node.config, _targetConfig);
                    }

                    if (distToGoal >= bestDist) {
                        return;
                    }
                }
            } while (!_bestPath.compareAndSet(currentBestPath, link));
        }

        private void updateChildren(Link newParent, Link oldParent, double radius) {
            assert newParent.node == oldParent.node : "updating links of different nodes";
            assert oldParent.isExpired() : "updating non-expired link";
            assert newParent.pathDist <= oldParent.pathDist : "updating to longer path";

            for (;;) {
                Link oldChild = oldParent.removeFirstChild();

                if (oldChild == null) {
                    // done.

                    if (newParent.isExpired()) {
                        oldParent = newParent;
                        newParent = oldParent.node.link.get();
                        continue;
                    }

                    return;
                }

                if (oldChild.isExpired()) {
                    assert _threadCount > 1;
                    continue;
                }

                Node node = oldChild.node;

                if (node.link.get().parent.get().node != oldParent.node) {
                    continue;
                }

                Link newChild = node.setLink(oldChild, oldChild.linkDist, newParent);

                if (newChild != null) {
                    updateChildren(newChild, oldChild, radius);
                    updateBestPath(newChild, radius);
                } else {
                    assert _threadCount > 1;
                    assert node.link.get() != oldChild;
                }
            }
        }

        private void rewire(Link oldLink, double linkDist, Node newParent, double radius) {
            assert oldLink.parent != null;
            assert oldLink.parent.get().node != newParent;

            Node node = oldLink.node;

            Link parentLink = newParent.link.get();

            double pathDist = parentLink.pathDist + linkDist;

            // check if rewiring would create a shorter path
            if (pathDist >= oldLink.pathDist) {
                return;
            }

            // check if rewiring is possible
            if (!_robotModel.link(oldLink.node.config, newParent.config)) {
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
                        updateChildren(parentLink.node.link.get(), parentLink, radius);
                    }

                    // Setting newLink expires oldLink but doesn not remove
                    // it from its parent. Here we do a little cleanup.
                    // We do it after the expired parent check since the parent
                    // will likely have already cleaned this up, and this call
                    // will be O(1) instead of O(n)
                    if (!oldLink.parent.get().removeChild(oldLink)) {
                        assert _threadCount > 1 : "concurrent update running with 1 thread";
                    }

                    return;
                }

                assert _threadCount > 1 : "concurrent update running with 1 thread";

                Link updatedOldLink = node.link.get();

                assert updatedOldLink != oldLink;

                oldLink = updatedOldLink;

            } while (pathDist < oldLink.pathDist);
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

            n.link = value.link.get();
            n.linkDist = dist;
            n.pathDist = n.link.pathDist + dist;
        }

        /**
         * Checks if a configuration is a goal configuration. If we are
         * searching for a path to a target goal configuration, this method
         * always returns false, since we will check the configuration can
         * extend to the target later.
         *
         * @param config the configuration to test
         * @return true if the configuration is in the goal region
         */
        private boolean inGoal(double[] config) {
            return _targetConfig == null && _robotModel.goal(config);
        }

        private void randomize(double[] config) {
            for (int i = _dimensions; --i >= 0;) {
                config[i] = _random.nextDouble() * (_sampleMax[i] - _sampleMin[i])
                        + _sampleMin[i];
            }
        }

        private void steer(double[] newConfig, double[] nearConfig, double t) {
            for (int i = _dimensions; --i >= 0;) {
                newConfig[i] = nearConfig[i] + (newConfig[i] - nearConfig[i]) * t;
            }
        }

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
                // nothing with in radius
                Node nearest = _kdTraversal.nearest(newConfig);
                double distToNearest = _kdTraversal.distToLastNearest();

                assert radius < distToNearest;

                steer(newConfig, nearest.config, radius / distToNearest);

                if (!_robotModel.clear(newConfig)) {
                    return false;
                }

                if (!_robotModel.link(nearest.config, newConfig)) {
                    return false;
                }

                // This should be radius, but might be off slightly so we
                // recalculate just to be safe.
                distToNearest = _kdModel.dist(newConfig, nearest.config);

                Node newNode = new Node(
                        newConfig, inGoal(newConfig), distToNearest, nearest.link.get());

                updateBestPath(newNode.link.get(), radius);

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

                if (!_robotModel.link(link.node.config, newConfig)) {
                    // help GC
                    _nearList[i].link = null;
                    continue;
                }

                // Found a linkable configuration. Create the node
                // and link it in here.

                Node newNode = new Node(
                        newConfig, inGoal(newConfig), _nearList[i].linkDist, link);

                updateBestPath(newNode.link.get(), radius);

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
                // worker initialization (normally would occur in constructor
                // but put here it will be run on its own thread in parallel)
                _robotModel = _robotModelProvider.get();
                _sampleMin = new double[_kdModel.dimensions()];
                _sampleMax = new double[_kdModel.dimensions()];

                _kdModel.getBounds(_sampleMin, _sampleMax);

                if (_perThreadRegionSampling) {
                    double min = _sampleMin[_regionSplitAxis];
                    double t = (_sampleMax[_regionSplitAxis] - min) / _threadCount;
                    _sampleMin[_regionSplitAxis] = min + _workerNo * t;
                    if (_workerNo + 1 < _threadCount) {
                        _sampleMax[_regionSplitAxis] = min + (_workerNo + 1) * t;
                    }
                }

                _kdTraversal = _kdTree.newTraversal();
                _random = _randomProvider.get();

                generateSamples();

            } finally {
                _doneLatch.countDown();
            }
        }
    }

}
