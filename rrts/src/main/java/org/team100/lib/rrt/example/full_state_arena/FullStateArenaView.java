package org.team100.lib.rrt.example.full_state_arena;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import javax.swing.JComponent;
import javax.swing.SwingUtilities;

import org.team100.lib.example.Arena;
import org.team100.lib.geom.Obstacle;
import org.team100.lib.graph.LinkInterface;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNode;
import org.team100.lib.index.KDTree;
import org.team100.lib.planner.Runner;
import org.team100.lib.space.Path;


/**
 * note this ignores model.dimensions since the plotted dimensions and model
 * dimensions aren't the same.
 * 
 * TODO: make both x/y and xdot/ydot pics
 */
public class FullStateArenaView extends JComponent {
    private static final boolean DEBUG = false;

    private final Runner _rrtStar;
    private final Arena _robotModel;

    private Image _backgroundImage;
    private Path _bestPath = null;

    private final NumberFormat _integerFormat = DecimalFormat.getIntegerInstance();

    private KDNode<Node> _T_a;
    private KDNode<Node> _T_b;

    public FullStateArenaView(Arena arena, Runner rrtStar, KDNode<Node> T_a,  KDNode<Node> T_b) {
        _rrtStar = rrtStar;
        _robotModel = arena;
        _T_a = T_a;
        _T_b = T_b;
    }

    @Override
    protected void paintComponent(Graphics graphics) {
        doPaint((Graphics2D) graphics, getSize());

        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                repaint();
            }
        });
    }

    public void doPaint(Graphics2D g, Dimension size) {
        if (DEBUG)
            System.out.println("doPaint");
        Arena robotModel = _robotModel;
        double[] min = robotModel.getMin();
        double[] max = robotModel.getMax();

        Path bestPath = _rrtStar.getBestPath();

        // if (_backgroundImage == null ||
        // _backgroundImage.getWidth(null) != size.width ||
        // _backgroundImage.getHeight(null) != size.height ||
        // Path.isBetter(bestPath, _bestPath)) {
        // createBGImage(min, max, size, bestPath);
        // _bestPath = bestPath;
        // }
        if (Path.isBetter(bestPath, _bestPath))
            _bestPath = bestPath;
        createBGImage(min, max, size, _bestPath);

        g.drawImage(_backgroundImage, 0, 0, null);

        AffineTransform savedTransform = g.getTransform();

        g.setTransform(savedTransform);

        g.setColor(Color.WHITE);
        FontMetrics fm = g.getFontMetrics();
        String count = _integerFormat.format(_rrtStar.getStepNo());
        g.drawString(count, 4, 4 + fm.getAscent());
        g.setColor(Color.BLACK);
        g.drawString(count, 3, 3 + fm.getAscent());
    }

    /** min and max are (x xdot y ydot) */
    private void createBGImage(double[] min, double[] max, Dimension size, Path link) {
        _backgroundImage = createImage(size.width, size.height);

        Graphics2D g = (Graphics2D) _backgroundImage.getGraphics();
        AffineTransform savedTransform = g.getTransform();

        double scale = setupGraphics(min, max, size, g);

        g.setColor(Color.WHITE);
        g.fillRect(0, 0, size.width, size.height);

        // obstacles
        g.setStroke(new BasicStroke(0f));
        // g.setColor(new Color(0x8888ff));
        for (Obstacle obstacle : _robotModel.obstacles()) {
            g.setColor(obstacle.color());
            g.fill(obstacle.shape());
        }

        renderRRTTree(g);

        renderPaths(link, g, scale);

        g.setTransform(savedTransform);
        g.dispose();
    }

    private static final boolean renderTree = true;

    public void renderRRTTree(Graphics2D g) {
        if (!renderTree)
            return;
        if (DEBUG)
            System.out.println("renderRRTTree");
        Line2D.Double line = new Line2D.Double();

        // List<Node> nodesA = _rrtStar.getNodesA();
        for (Node node : KDTree.values( _T_a)) {
            LinkInterface incoming = node.getIncoming();
            if (incoming != null) {
                Node parent = incoming.get_source();
                double[] n = node.getState();
                double[] p = parent.getState();
                if (DEBUG)
                    System.out.printf("A node %s parent %s\n"
                            + Arrays.toString(n), Arrays.toString(p));
                g.setColor(Color.GREEN);
                line.setLine(n[0], n[2], p[0], p[2]);
                g.draw(line);
            }
        }
        // List<Node> nodesB = _rrtStar.getNodesB();
        for (Node node : KDTree.values(_T_b)) {
            LinkInterface incoming = node.getIncoming();
            if (incoming != null) {
                Node parent = incoming.get_source();
                double[] n = node.getState();
                double[] p = parent.getState();
                if (DEBUG)
                    System.out.printf("B node %s parent %s\n"
                            + Arrays.toString(n), Arrays.toString(p));
                g.setColor(Color.RED);
                line.setLine(n[0], n[2], p[0], p[2]);
                g.draw(line);
            }
        }
    }

    private void renderPaths(Path path, Graphics2D g, double scale) {
        if (path == null) {
            return;
        }

        Line2D.Double line = new Line2D.Double();
        g.setStroke(new BasicStroke((float) (5 / scale)));

        List<double[]> statesA = path.getStatesA();
        if (statesA.size() > 1) {
            Iterator<double[]> pathIter = statesA.iterator();
            double[] prev = pathIter.next();
            while (pathIter.hasNext()) {
                double[] curr = pathIter.next();
                g.setColor(Color.GREEN);
                line.setLine(prev[0], prev[2], curr[0], curr[2]);
                g.draw(line);
                prev = curr;
            }
        }
        List<double[]> statesB = path.getStatesB();
        if (statesB.size() > 1) {
            Iterator<double[]> pathIter = statesB.iterator();
            double[] prev = pathIter.next();
            while (pathIter.hasNext()) {
                double[] curr = pathIter.next();
                g.setColor(Color.RED);
                line.setLine(prev[0], prev[2], curr[0], curr[2]);
                g.draw(line);
                prev = curr;
            }
        }

        double[] nA = statesA.get(statesA.size() -  1);
        double[] nB = statesB.get(0);
        if (nA != null && nB != null) {
           // System.out.printf("LINK [%5.3f %5.3f] to [%5.3f %5.3f]\n",
            //        nA[0], nA[2], nB[0], nB[2]);
            g.setColor(Color.BLACK);
            line.setLine(nA[0], nA[2], nB[0], nB[2]);
            g.draw(line);
        } else {
            System.out.println("NULLS");
        }

    }

    /** min and max are (x xdot y ydot) */
    private double setupGraphics(double[] min, double[] max, Dimension size, Graphics2D g) {
        g.setRenderingHint(
                RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);

        g.translate(min[0], min[2]);
        double scale = Math.min(
                size.width / (max[0] - min[0]),
                size.height / (max[2] - min[2]));
        g.scale(scale, scale);
        g.setStroke(new BasicStroke((float) (0.25 / scale)));
        return scale;
    }
}
