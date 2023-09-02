package org.team100.lib.rrt.example.swingup;
import java.awt.BorderLayout;
import java.lang.reflect.InvocationTargetException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import org.team100.lib.example.Arena;
import org.team100.lib.planner.Runner;
import org.team100.lib.rrt.RRTStar5;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Two dimensional non-Euclidean */
public class PendulumFrame extends JFrame {
    public PendulumFrame(Arena<N2> arena, Runner<N2> rrtStar) {
        super("Pendulum RRT*");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(new PendulumView(arena, rrtStar));
    }

    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        Matrix<N2,N1> init = new Matrix<>(Nat.N2(),Nat.N1(), new double[] { 0, 0 });
        Matrix<N2,N1> goal = new Matrix<>(Nat.N2(),Nat.N1(), new double[] { Math.PI, 0 });
        double gravity = 9.81;
        final Arena<N2> arena = new PendulumArena2(init, goal, gravity);
        final RRTStar5<Arena<N2>> worker = new RRTStar5<>(arena, new Sample<>(arena), 1);
        final Runner<N2> rrtStar = new Runner<>(worker);

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
        //worker.SwapTrees();
        rrtStar.runForDurationMS(1000);
        //rrtStar.runSamples(3000);
        Path<N2> bestPath = rrtStar.getBestPath();
        if (bestPath != null) {
            System.out.println(bestPath);
        }
        System.out.println("done");
    }
}
