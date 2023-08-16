package edu.unc.robotics.prrts;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.unc.robotics.prrts.kdtree.KDModel;
import edu.unc.robotics.prrts.kdtree.KDTree;
import edu.unc.robotics.prrts.tree.Link;
import edu.unc.robotics.prrts.tree.Node;

/**
 * PRRTStar
 *
 * @author jeffi
 */
public class PRRTStar {
    private static final Logger _log = Logger.getLogger(PRRTStar.class.getName());
    
    private final KDModel _kdModel;
    private final RobotModel _robotModel;
    private final double _gamma; // step size
    private final KDTree<Node> _kdTree;
    private final AtomicInteger _stepNo = new AtomicInteger(0);
    private final AtomicBoolean _done = new AtomicBoolean(false);
    private final CountDownLatch _doneLatch;
    private final int _threadCount;
    private final AtomicReference<Link> _bestPath = new AtomicReference<Link>();

    public PRRTStar(
            KDModel kdModel,
            RobotModel robotModel,
            double[] init,
            double gamma,
            int threadCount) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        if (threadCount < 1) {
            throw new IllegalArgumentException("thread count must be >= 1");
        }
        _kdModel = kdModel;
        _robotModel = robotModel;
        _kdTree = new KDTree<Node>(kdModel, init, new Node(init, false));
        _gamma = gamma;
        _threadCount = threadCount;
        _doneLatch = new CountDownLatch(_threadCount);

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
        double pathDist = link.get_pathDist();
        for (; link != null; link = link.get_parent()) {
            configs.add(link.get_node().get_config());
        }
        Collections.reverse(configs);
        return new Path(pathDist, configs);
    }

    public Path runForDurationMS(long milliseconds) {
        return run(Integer.MAX_VALUE, milliseconds);
    }

    public Path runSamples(int samples) {
        return run(samples, 0);
    }

    private Path run(int sampleLimit, long timeLimitMS) {

        if (timeLimitMS < 0) {
            throw new IllegalArgumentException("invalid duration, must be >= 0");
        }
        long timeLimitNS = timeLimitMS * 1000000;

        long startTime = System.nanoTime();

        Worker[] workers = new Worker[_threadCount];
        for (int i = 0; i < _threadCount; ++i) {
            workers[i] = new Worker(
                    _kdModel,
                    _kdTree.newTraversal(),
                    _robotModel,
                    _gamma, i,
                    _threadCount,
                    timeLimitNS,
                    startTime,
                    sampleLimit,
                    _doneLatch,
                    _stepNo,
                    _bestPath,
                    _done);
        }

        ThreadGroup threadGroup = Thread.currentThread().getThreadGroup();
        Thread[] threads = new Thread[_threadCount];
        for (int i = 1; i < _threadCount; ++i) {
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
                for (int i = 1; i < _threadCount; ++i) {
                    threads[i].join();
                }
            }
        } catch (InterruptedException e) {
            _log.log(Level.WARNING, "Interrupted", e);
        }

        return getBestPath();
    }

}
