package edu.unc.robotics.prrts.example.arena;

import java.awt.BorderLayout;
import java.lang.reflect.InvocationTargetException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import org.team100.lib.planner.Runner;
import org.team100.lib.planner.Solver;
import org.team100.lib.rrt.RRTStar;
import org.team100.lib.rrt.RRTStar2;
import org.team100.lib.space.Sample;

public class ArenaFrame extends JFrame {

    public ArenaFrame(HolonomicArena arena, Runner rrtStar) {
        super("RRT*");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(new ArenaView(arena, rrtStar));
    }

    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        int ct = 10;
        double steps = 0;
        double steps2 = 0;
        double dist = 0;
        double dist2 = 0;
        for (int i = 0; i < ct; ++i) {

            final HolonomicArena arena = new HolonomicArena(6);
            final RRTStar<HolonomicArena> rrtstar = new RRTStar<>(arena, new Sample(arena), 6);

            final Runner runner = new Runner(rrtstar);
            run(arena, runner, rrtstar);
            final RRTStar2<HolonomicArena> rrtstar2 = new RRTStar2<>(arena, new Sample(arena), 6);
            final Runner runner2 = new Runner(rrtstar2);
            run(arena, runner2, rrtstar2);

            System.out.printf("1 steps %d distance %5.2f --- 2 steps %d distance %5.2f\n",
                    runner.getStepNo(), runner.getBestPath().getDistance(),
                    runner2.getStepNo(), runner2.getBestPath().getDistance());
            steps += runner.getStepNo();
            steps2 += runner2.getStepNo();
            dist += runner.getBestPath().getDistance();
            dist2 += runner2.getBestPath().getDistance();
        }
        System.out.println("means");
        System.out.printf("1 steps %7.2f distance %5.2f --- 2 steps %7.2f distance %5.2f\n",
                steps / ct, dist / ct, steps2 / ct, dist2 / ct);
    }

    private static void run(HolonomicArena arena, Runner runner, Solver rrtstar)
            throws InvocationTargetException, InterruptedException {

        // SwingUtilities.invokeAndWait(new Runnable() {
        //     @Override
        //     public void run() {
        //         ArenaFrame frame = new ArenaFrame(arena, runner);
        //         frame.setSize(1600, 800);
        //         frame.setVisible(true);
        //         frame.repaint();
        //     }
        // });

        runner.runForDurationMS(100);
        // runner.runSamples(500);

    }
}
