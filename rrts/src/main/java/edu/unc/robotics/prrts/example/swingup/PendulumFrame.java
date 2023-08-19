package edu.unc.robotics.prrts.example.swingup;

import java.lang.reflect.InvocationTargetException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import java.awt.BorderLayout;

import edu.unc.robotics.prrts.PRRTStar;
import edu.unc.robotics.prrts.Path;

public class PendulumFrame extends JFrame {
    public PendulumFrame(PendulumArena arena, PRRTStar rrtStar) {
        super("Pendulum RRT*");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(new PendulumView(arena, rrtStar));

    }

    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        final PendulumArena arena = new PendulumArena(new double[] { Math.PI, 0 }, 9.81);
        double[] init = { 0, 0 };

        final PRRTStar rrtStar = new PRRTStar(arena, arena, init);

        SwingUtilities.invokeAndWait(new Runnable() {
            @Override
            public void run() {
                PendulumFrame frame = new PendulumFrame(arena, rrtStar);
                frame.setSize(600, 600);
                frame.setVisible(true);
                frame.repaint();
            }
        });

        // rrtStar.runForDurationMS(2, 10, 3000);
        rrtStar.runSamples(2, 20000);
        Path bestPath = rrtStar.getBestPath();
        System.out.println(bestPath);

    }

}
