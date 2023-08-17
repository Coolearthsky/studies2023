package edu.unc.robotics.prrts.example.arena;

import java.awt.BorderLayout;
import java.lang.reflect.InvocationTargetException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import edu.unc.robotics.prrts.PRRTStar;

/**
 * ArenaFrame
 *
 * @author jeffi
 */
public class ArenaFrame extends JFrame {

    public ArenaFrame(HolonomicArena arena, PRRTStar rrtStar) {
        super("RRT*");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(new ArenaView(arena, rrtStar));
    }

    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        final HolonomicArena arena = new HolonomicArena();
        double[] init = { 15.5, 6.75 };

        // 2 threads works better than 4
        final PRRTStar rrtStar = new PRRTStar(arena, arena, init);

        SwingUtilities.invokeAndWait(new Runnable() {
            @Override
            public void run() {
                ArenaFrame frame = new ArenaFrame(arena, rrtStar);
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
