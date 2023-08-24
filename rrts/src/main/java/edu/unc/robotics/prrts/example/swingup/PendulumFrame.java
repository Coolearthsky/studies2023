package edu.unc.robotics.prrts.example.swingup;

import java.lang.reflect.InvocationTargetException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import org.team100.lib.planner.Runner;
import org.team100.lib.planner.Solver;
import org.team100.lib.rrt.RRTStar;
import org.team100.lib.rrt.RRTStar4;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;

import java.awt.BorderLayout;

public class PendulumFrame extends JFrame {
    public PendulumFrame(PendulumArena arena, Runner rrtStar) {
        super("Pendulum RRT*");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(new PendulumView(arena, rrtStar));

    }

    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        final PendulumArena arena = new PendulumArena(new double[] { 0, 0 }, new double[] { Math.PI, 0 }, 9.81);
        final Solver worker = new RRTStar4<>(arena, new Sample(arena), 2);
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

        // rrtStar.runForDurationMS(3000);
        rrtStar.runSamples(20000);
        Path bestPath = rrtStar.getBestPath();
        System.out.println(bestPath);

    }

}
