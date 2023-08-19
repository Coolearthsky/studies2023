package edu.unc.robotics.prrts;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import edu.unc.robotics.prrts.kdtree.KDModel;
import edu.unc.robotics.prrts.kdtree.KDNode;
import edu.unc.robotics.prrts.kdtree.Util;
import edu.unc.robotics.prrts.tree.Link;
import edu.unc.robotics.prrts.tree.Node;

/**
 * PRRTStar
 *
 * @author jeffi
 */
public class PRRTStar {

    private final KDModel _kdModel;
    private final RobotModel _robotModel;
    private final AtomicInteger _stepNo;
    private final AtomicBoolean _done;
    private final AtomicReference<Link> _bestPath;
    private final KDNode<Node> _rootNode ;

    public PRRTStar(
            KDModel kdModel,
            RobotModel robotModel,
            double[] init) {
        _kdModel = kdModel;
        _robotModel = robotModel;
        _rootNode =new KDNode<Node>(init, new Node(init, false));
        _stepNo = new AtomicInteger(0);
        _done = new AtomicBoolean(false);
        _bestPath = new AtomicReference<Link>();
    }


    public int getStepNo() {
        return _stepNo.get();
    }

    public Iterable<Node> getNodes() {
        return Util.values(_rootNode);
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

        Node node = link.get_node();
        while (node != null) {
            configs.add(node.get_config());
            node = node.get_parent_node();
        }
        Collections.reverse(configs);
        return new Path(pathDist, configs);
    }

    public Path runForDurationMS(double gamma, long milliseconds) {
        return run(gamma, Integer.MAX_VALUE, milliseconds);
    }

    public Path runSamples(double gamma, int samples) {
        return run(gamma, samples, 0);
    }

    private Path run(double gamma, int sampleLimit, long timeLimitMS) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        if (timeLimitMS < 0) {
            throw new IllegalArgumentException("invalid duration, must be >= 0");
        }
        long timeLimitNS = timeLimitMS * 1000000;

        long startTime = System.nanoTime();

        Worker worker  = new Worker(
                    _kdModel,
                    _rootNode,
                    _robotModel,
                    new Sample(_kdModel),
                    gamma,
                    timeLimitNS,
                    startTime,
                    sampleLimit,
                    _stepNo,
                    _bestPath,
                    _done);

   
        worker.run();

        return getBestPath();
    }


    
}
