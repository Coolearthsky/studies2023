package edu.unc.robotics.prrts.example.arena;

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
import java.util.Iterator;

import javax.swing.JComponent;
import javax.swing.SwingUtilities;

import edu.unc.robotics.prrts.PRRTStar;
import edu.unc.robotics.prrts.Path;
import edu.unc.robotics.prrts.example.geom.Obstacle;
import edu.unc.robotics.prrts.tree.Node;

/**
 * ArenaView
 *
 * @author jeffi
 */
public class ArenaView extends JComponent {
    private final PRRTStar _rrtStar;
    private final HolonomicArena _robotModel;

    private static final Color[] COLORS = new Color[] {
            Color.BLACK, Color.BLUE, Color.MAGENTA, Color.GREEN
    };

    private Image _backgroundImage;
    private Path _bestPath = null;

    private final NumberFormat _integerFormat = DecimalFormat.getIntegerInstance();

    public ArenaView(HolonomicArena arena, PRRTStar rrtStar) {
        _rrtStar = rrtStar;
        _robotModel = arena;
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
        HolonomicArena robotModel = _robotModel;
        double[] min = new double[robotModel.dimensions()];
        double[] max = new double[robotModel.dimensions()];
        robotModel.getBounds(min, max);

        Path bestPath = _rrtStar.getBestPath();

        if (_backgroundImage == null ||
                _backgroundImage.getWidth(null) != size.width ||
                _backgroundImage.getHeight(null) != size.height ||
                Path.isBetter(bestPath, _bestPath)) {
            createBGImage(min, max, size, bestPath);
            _bestPath = bestPath;
        }

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

    private void renderRRTTree(Graphics2D g) {
        int dim = _robotModel.dimensions();
        Line2D.Double line = new Line2D.Double();

        for (Node node : _rrtStar.getNodes()) {
            Node parent = node.get_parent_node();
            if (parent != null) {
                double[] n = node.get_config();
                double[] p = parent.get_config();
                for (int i = 0; i < dim; i += 2) {
                    g.setColor(COLORS[i / 2]);
                    line.setLine(n[i], n[i + 1], p[i], p[i + 1]);
                    g.draw(line);
                }
            }
        }
    }

    private void renderPaths(Path path, Graphics2D g, double scale) {
        if (path == null) {
            return;
        }

        int dim = _robotModel.dimensions();
        Line2D.Double line = new Line2D.Double();
        g.setStroke(new BasicStroke((float) (5 / scale)));

        if (path.get_configs().size() > 1) {
            Iterator<double[]> pathIter = path.get_configs().iterator();
            double[] prev = pathIter.next();
            while (pathIter.hasNext()) {
                double[] curr = pathIter.next();
                for (int i = 0; i < dim; i += 2) {
                    g.setColor(brighter(COLORS[i / 2]));
                    line.setLine(prev[i], prev[i + 1], curr[i], curr[i + 1]);
                    g.draw(line);
                }
                prev = curr;
            }
        }
    }

    private double setupGraphics(double[] min, double[] max, Dimension size, Graphics2D g) {
        g.setRenderingHint(
                RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);

        g.translate(min[0], min[1]);
        double scale = Math.min(
                size.width / (max[0] - min[0]),
                size.height / (max[1] - min[1]));
        g.scale(scale, scale);
        g.setStroke(new BasicStroke((float) (0.5 / scale / _robotModel.dimensions())));
        return scale;
    }

    static Color brighter(Color color) {
        float[] hsb = Color.RGBtoHSB(color.getRed(), color.getGreen(), color.getBlue(), null);
        hsb[1] = Math.max(0.0f, hsb[1] - 0.25f);
        hsb[2] = Math.min(1.0f, hsb[2] + 0.25f);
        color = Color.getHSBColor(hsb[0], hsb[1], hsb[2]);
        return color;
    }
}
