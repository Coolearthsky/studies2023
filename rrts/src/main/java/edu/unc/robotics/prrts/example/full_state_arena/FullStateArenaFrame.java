package edu.unc.robotics.prrts.example.full_state_arena;

import java.awt.BorderLayout;
import java.lang.reflect.InvocationTargetException;
import java.util.Collections;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import org.team100.lib.graph.Graph;
import org.team100.lib.planner.Runner;
import org.team100.lib.planner.Solver;
import org.team100.lib.rrt.RRTStar6;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;

public class FullStateArenaFrame extends JFrame {
    private static final boolean DEBUG = false;

    public FullStateArenaFrame(FullStateArenaView view) {
        super("RRT*");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(view);
    }

    public static void main(String[] args) throws InterruptedException, InvocationTargetException {

        double[] init = new double[] { 0, 0 };
        // double[] goal = new double[] { Math.PI, 0 };

        final FullStateHolonomicArena arena = new FullStateHolonomicArena(6);
        final Solver solver = new RRTStar6<>(arena, new Sample(arena), 6);
        final Runner runner = new Runner(solver);
        final FullStateArenaView view = new FullStateArenaView(arena, runner);
        final FullStateArenaFrame frame = new FullStateArenaFrame(view);

        SwingUtilities.invokeAndWait(new Runnable() {
            @Override
            public void run() {
                frame.setSize(1600, 800);
                frame.setVisible(true);
                frame.repaint();
            }
        });

        // runner.runForDurationMS(20000);
        runner.runSamples(10000);

        Path bestPath = runner.getBestPath();
        if (bestPath == null) {
            System.out.println("failed to find path");
        } else {
            System.out.println("found path");
            List<double[]> states = bestPath.getStates();

            if (!same(init, states.get(0))) {
                Collections.reverse(states);
                bestPath = new Path(bestPath.getDistance(), states);
            }
            System.out.println(bestPath);
        }
        System.out.println("done");
        frame.repaint();
        view.repaint();

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
