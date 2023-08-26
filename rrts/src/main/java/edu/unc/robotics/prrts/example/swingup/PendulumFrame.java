package edu.unc.robotics.prrts.example.swingup;

import java.awt.BorderLayout;
import java.lang.reflect.InvocationTargetException;
import java.util.Collections;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import org.team100.lib.planner.Runner;
import org.team100.lib.rrt.RRTStar5;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;

public class PendulumFrame extends JFrame {
    public PendulumFrame(Arena arena, Runner rrtStar) {
        super("Pendulum RRT*");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(new PendulumView(arena, rrtStar));

    }

    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        double[] init = new double[] { 0, 0 };
        double[] goal = new double[] { Math.PI, 0 };
        double gravity = 9.81;
        final Arena arena = new PendulumArena2(init, goal, gravity);
        final RRTStar5<Arena> worker = new RRTStar5<>(arena, new Sample(arena), 2);
        final Runner rrtStar = new Runner(worker);

        SwingUtilities.invokeAndWait(new Runnable() {
            @Override
            public void run() {
                PendulumFrame frame = new PendulumFrame(arena, rrtStar);
                frame.setSize(600, 600);
                frame.setVisible(true);
                frame.repaint();
            }
        });
        // it should work both ways, time-reversed in this case:
       // worker.SwapTrees();
        // rrtStar.runForDurationMS(2000);
        rrtStar.runSamples(100000);
        Path bestPath = rrtStar.getBestPath();
        List<double[]> states = bestPath.getStates();

        if (!same(init, states.get(0))) {
            Collections.reverse(states);
            bestPath = new Path(bestPath.getDistance(), states);
        }
        System.out.println(bestPath);
    }

    static boolean same(double[] a, double[] b) {
        if (a.length != b.length)
            return false;
        for (int i = 0; i < a.length; ++i) {
            if (Math.abs(a[i] - b[i]) > 0.0001)
                return false;
        }
        return true;
    }

}
