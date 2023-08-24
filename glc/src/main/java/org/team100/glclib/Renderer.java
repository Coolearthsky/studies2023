package org.team100.glclib;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.lang.reflect.InvocationTargetException;

import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;

public class Renderer {
    public static class Frame extends JFrame {
        public Frame(InterpolatingPolynomial solution) {
            super("GLC");
            setDefaultCloseOperation(EXIT_ON_CLOSE);
            getContentPane().setLayout(new BorderLayout());
            getContentPane().add(new View(solution));
        }
    }

    public static void render(InterpolatingPolynomial solution) {
        try {
            SwingUtilities.invokeAndWait(new Runnable() {
                @Override
                public void run() {
                    Frame frame = new Frame(solution);
                    frame.setSize(800, 600);
                    frame.setVisible(true);
                    frame.repaint();
                }
            });
        } catch (InvocationTargetException | InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static class View extends JComponent {
        InterpolatingPolynomial solution;

        public View(InterpolatingPolynomial solution) {
            this.solution = solution;
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

        }

    }

}
