package org.team100.lib.planner;

import java.util.List;

import org.team100.lib.graph.Node;
import org.team100.lib.space.Path;

import edu.wpi.first.math.Num;

/**
 * Runs the specified steps or time.
 */
public class Runner<States extends Num> {
    private static final boolean DEBUG = false;

    private final Solver<States> _solver;
    private int _stepNo;

    public Runner(Solver<States> solver) {
        _solver = solver;
        // since we use stepNo for radius, it can't be zero
        _stepNo = 1;
    }

    public void runForDurationMS(long milliseconds) {
        if (milliseconds <= 0) {
            throw new IllegalArgumentException("invalid duration, must be > 0");
        }
        run(Integer.MAX_VALUE, milliseconds * 1000000);
    }

    public void runSamples(int samples) {
        if (samples <= 0) {
            throw new IllegalArgumentException("invalid sample count, must be > 0");
        }
        run(samples, 0);
    }

    /** For listeners. */
    public int getStepNo() {
        return _stepNo;
    }

    /** For listeners. */
    public List<Node<States>> getNodesA() {
        return _solver.getNodesA();
    }

    /** For listeners. */
    public List<Node<States>> getNodesB() {
        return _solver.getNodesB();
    }

    /** For listeners. */
    public Path<States> getBestPath() {
        return _solver.getBestPath();
    }

    /////////////////////////////////////////

    private void run(int sampleLimit, long timeLimitNS) {
        long startTime = System.nanoTime();
        int actualLimit = 0;
        while (true) {
            if (DEBUG)
                System.out.println("counter: " + actualLimit);
            actualLimit += 1;
            if (actualLimit > 10000)
                break;
            _solver.setStepNo(_stepNo);
            if (_solver.step() > 0) {
                _stepNo++;
                if (_stepNo > sampleLimit) {
                    return;
                }
                // try {
                //     Thread.sleep(2000);
                // } catch (InterruptedException e) {
                //     // TODO Auto-generated catch block
                //     e.printStackTrace();
                // }
            }

            if (timeLimitNS > 0) {
                long now = System.nanoTime();
                if (now - startTime > timeLimitNS) {
                    return;
                }
            }
        }
    }
}
