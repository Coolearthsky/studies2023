package team100;

import team100.geometry.Point;
import team100.geometry.Polygon;
import team100.rrt.Rrt;
import team100.rrt.RrtPanel;
import team100.visibilitygraph.VisibilityGraph;
import team100.visibilitygraph.VisibilityGraphPanel;

public class RrtDemo {
    private static final int cX = 300;
    private static final int cY = 250;

    public void run() {
        Rrt vg = buildDemoGraph();
        RrtPanel vgp = new RrtPanel(vg);

        // rebuilds the entire field every time, which is surely slow?
        double graphTime = 0;
        double astarTime = 0;
        double totalTime = 0;
        int iterations = 5000;

        for (double i = 0; i < 2 * Math.PI; i += 2 * Math.PI / iterations) {
            long startTime = System.nanoTime();

            double x = cX * (1 + Math.cos(i));
            double y = cY * (1 + Math.sin(i));

            Point startPoint = new Point((int) x, (int) y);

            // make a new graph
            vg = buildDemoGraph(startPoint);
            long endTime = System.nanoTime();
            graphTime += ((double) endTime - startTime) / 1e9;
            vgp.visibilityGraph = vg;

            // actually compute the shortest path
            vgp.prepareForRender();

            long end2Time = System.nanoTime();
            astarTime += ((double) end2Time - endTime) / 1e9;
            totalTime += ((double) end2Time - startTime) / 1e9;

            // turn off repainting for now
            vgp.repaint();

        }
        System.out.printf("iterations = %d\n", iterations);
        System.out.printf("graphTime = %f, %f per iteration\n", graphTime, graphTime / iterations);
        System.out.printf("astarTime = %f, %f per iteration\n", astarTime, astarTime / iterations);
        System.out.printf("totalTime = %f, %f per iteration\n", totalTime, totalTime / iterations);

    }

    private static Rrt buildDemoGraph(Point start) {
        Polygon p1 = new Polygon(
                new Point(70, 30),
                new Point(20, 60),
                new Point(80, 90));

        Polygon p2 = new Polygon(
                new Point(540, 310),
                new Point(520, 350),
                new Point(490, 350),
                new Point(460, 310),
                new Point(480, 270),
                new Point(520, 270));

        Polygon p3 = new Polygon(
                new Point(220, 170),
                new Point(180, 180),
                new Point(230, 300),
                new Point(270, 290));

        Polygon p4 = new Polygon(
                new Point(290, 330),
                new Point(220, 330),
                new Point(235, 360),
                new Point(270, 360));

        Polygon p5 = new Polygon(
                new Point(60, 120),
                new Point(160, 200),
                new Point(90, 90));

        Polygon p6 = new Polygon(
                new Point(300, 150),
                new Point(480, 200),
                new Point(400, 300));

        Polygon p7 = new Polygon(
                new Point(400, 160),
                new Point(460, 40),
                new Point(300, 20),
                new Point(300, 120));

        Point end = new Point(560, 50);

        return new VisibilityGraph(start, end, p1, p2, p3, p4, p5, p6, p7);
    }

    private static VisibilityGraph buildDemoGraph() {
        return buildDemoGraph(new Point(100, 360));
    }
}
