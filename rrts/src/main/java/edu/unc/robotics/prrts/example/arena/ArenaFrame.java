package edu.unc.robotics.prrts.example.arena;

import java.awt.BorderLayout;
import java.lang.reflect.InvocationTargetException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import edu.unc.robotics.prrts.Runner;
import edu.unc.robotics.prrts.Sample;
import edu.unc.robotics.prrts.RRTStar;

/**
 * ArenaFrame
 *
 * @author jeffi
 */
public class ArenaFrame extends JFrame {

    public ArenaFrame(HolonomicArena arena, Runner rrtStar) {
        super("RRT*");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(new ArenaView(arena, rrtStar));
    }

    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        final HolonomicArena arena = new HolonomicArena();
        final RRTStar<HolonomicArena> rrtstar = new RRTStar<>(arena, new Sample(arena), 6);
        final Runner runner = new Runner(rrtstar);

        SwingUtilities.invokeAndWait(new Runnable() {
            @Override
            public void run() {
                ArenaFrame frame = new ArenaFrame(arena, runner);
                frame.setSize(1600, 800);
                frame.setVisible(true);
                frame.repaint();
            }
        });

        runner.runForDurationMS(50);
        //runner.runSamples(500);
    }
}
