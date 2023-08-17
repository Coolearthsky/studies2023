package edu.unc.robotics.prrts.example.swingup;

import java.lang.reflect.InvocationTargetException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import java.awt.BorderLayout;

import edu.unc.robotics.prrts.PRRTStar;

public class PendulumFrame extends JFrame {
    public PendulumFrame(PendulumArena arena, PRRTStar rrtStar) {
        super("Pendulum RRT*");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(new PendulumView(arena, rrtStar));

    }


    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        final PendulumArena arena = new PendulumArena();
        double[] init = { 15.5, 6.75 };

        // 2 threads works better than 4
        final PRRTStar rrtStar = new PRRTStar(arena, arena, init);

        SwingUtilities.invokeAndWait(new Runnable() {
            @Override
            public void run() {
                PendulumFrame frame = new PendulumFrame(arena, rrtStar);
                frame.setSize(1600, 800);
                frame.setVisible(true);
                frame.repaint();
            }
        });

        Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
        Thread.currentThread().getThreadGroup().setMaxPriority(Thread.MIN_PRIORITY);

        rrtStar.runForDurationMS(2, 6.0, 30);
        //rrtStar.runSamples(2, 6.0, 500);
    }
    
}
