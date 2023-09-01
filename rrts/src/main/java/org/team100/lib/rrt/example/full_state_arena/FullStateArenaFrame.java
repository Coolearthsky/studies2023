package org.team100.lib.rrt.example.full_state_arena;

import java.awt.BorderLayout;
import java.lang.reflect.InvocationTargetException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNode;
import org.team100.lib.planner.Runner;
import org.team100.lib.planner.Solver;
import org.team100.lib.rrt.RRTStar6;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;

public class FullStateArenaFrame extends JFrame {
    public FullStateArenaFrame(FullStateArenaView view) {
        super("RRT*");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(view);
    }

    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        final FullStateHolonomicArena arena = new FullStateHolonomicArena();
        KDNode<Node> T_a = new KDNode<Node>(new Node(arena.initial()));
        KDNode<Node> T_b = new KDNode<Node>(new Node(arena.goal()));
        final Solver solver = new RRTStar6<>(arena, new Sample(arena), 3, T_a, T_b);
        final Runner runner = new Runner(solver);
        final FullStateArenaView view = new FullStateArenaView(arena, runner, T_a, T_b);
        final FullStateArenaFrame frame = new FullStateArenaFrame(view);

        SwingUtilities.invokeAndWait(new Runnable() {
            @Override
            public void run() {
                frame.setSize(1600, 800);
                frame.setVisible(true);
                frame.repaint();
            }
        });

        runner.runForDurationMS(100000);
        //runner.runSamples(5000);

        Path bestPath = runner.getBestPath();
        if (bestPath == null) {
            System.out.println("failed to find path");
        } else {
            System.out.println("found path");
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
