package edu.unc.robotics.prrts.example.arena;

import java.awt.BorderLayout;
import java.lang.reflect.InvocationTargetException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import edu.unc.robotics.prrts.ArrayState;
import edu.unc.robotics.prrts.PRRTStar;

/**
 * ArenaFrame
 *
 * @author jeffi
 */
public class ArenaFrame extends JFrame {

    public ArenaFrame(HolonomicArena arena, PRRTStar<ArrayState> rrtStar) {
        super("RRT*");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(new ArenaView(arena, rrtStar));
    }


    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        final HolonomicArena arena = new HolonomicArena();
        ArrayState init = new ArrayState(new double[] {7.0, 1.0, 8, 8, 9, 1, 1, 9});

        final PRRTStar<ArrayState> rrtStar = new PRRTStar<>(arena, () -> arena, init);

        rrtStar.setGamma(6.0);

        SwingUtilities.invokeAndWait(new Runnable() {
            @Override
            public void run() {
                ArenaFrame frame = new ArenaFrame(arena, rrtStar);
                frame.setSize(800, 800);
                frame.setVisible(true);

                frame.repaint();
            }
        });

        Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
        Thread.currentThread().getThreadGroup().setMaxPriority(Thread.MIN_PRIORITY);

        // 2 threads works better than 4
        rrtStar.runForDuration(2, 20);
    }
}
