package edu.unc.robotics.prrts.kdtree;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

public class TestTrees {
    private static class MyKDModel implements KDModel {
        @Override
        public int dimensions() {
            return 2;
        }

        @Override
        public double[] getMin() {
            return new double[]{0,0};
        }

        @Override
        public double[] getMax() {
            return new double[]{1,1};
        }

        @Override
        public double dist(double[] start, double[] end) {
            return Math.sqrt(Math.pow(start[0] - end[0], 2) + Math.pow(start[1] - end[1], 2));
        }

        @Override
        public void steer(double[] nearConfig, double[] newConfig, double dist) {
            throw new UnsupportedOperationException("Unimplemented method 'steer'");
        }
    }

    @Test
    void treeTest() {
        KDModel m = new MyKDModel();
        double[] init = new double[] { 0, 0 };
        KDNode<String> rootNode = new KDNode<String>(init, "root");
        Util.insert(m, rootNode, new double[] { 0.5, 0.5 }, "child1");
        Util.insert(m, rootNode, new double[] { 0.5, 075 }, "child2");
        Util.insert(m, rootNode, new double[] { 0.5, 0.25 }, "child3");

        List<String> nearList = new ArrayList<String>();
        List<Double> distList = new ArrayList<Double>();
        Util.near(m, rootNode, new double[] { 0.25, 0.25 }, 0.5, (value, dist) -> {
            nearList.add(value);
            distList.add(dist);
        });
        assertEquals(3, nearList.size());
        assertEquals("root", nearList.get(0));
        assertEquals("child1", nearList.get(1));
        assertEquals("child3", nearList.get(2));
        assertEquals(0.353, distList.get(0), 0.001);
        assertEquals(0.353, distList.get(1), 0.001);
        assertEquals(0.25, distList.get(2), 0.001);
    }

    @Test
    void treeTest2() {
        KDModel m = new MyKDModel();
        double[] init = new double[] { 0, 0 };
        KDNode<String> rootNode = new KDNode<String>(init, "root");
        Util.insert(m, rootNode, new double[] { 0.5, 0.5 }, "child1");
        Util.insert(m, rootNode, new double[] { 0.5, 075 }, "child2");
        Util.insert(m, rootNode, new double[] { 0.5, 0.25 }, "child3");

        List<String> nearList = new ArrayList<String>();
        List<Double> distList = new ArrayList<Double>();
        // small radius
        Util.near(m, rootNode, new double[] { 0.25, 0.25 }, 0.1, (value, dist) -> {
            nearList.add(value);
            distList.add(dist);
        });
        assertEquals(0, nearList.size());

        KDNearNode<String> nearest = Util.nearest(m, rootNode, new double[] { 0.25, 0.25 });
        assertEquals("child3", nearest._nearest);
        assertEquals(0.25, nearest._dist, 0.001);
    }

    @Test
    void treeTest3() {
        // are these asserts actually valid?
        KDModel m = new MyKDModel();
        double[] init = new double[] { 0, 0 };
        KDNode<String> rootNode = new KDNode<String>(init, "root");

        Util.insert(m, rootNode, new double[] { 0.5, 0.5 }, "child1");
        Util.insert(m, rootNode, new double[] { 0.5, 075 }, "child2");

        // if the other view does the insert, the child is still found
        Util.insert(m, rootNode, new double[] { 0.5, 0.25 }, "child3");

        List<String> nearList = new ArrayList<String>();
        List<Double> distList = new ArrayList<Double>();
        // small radius
        Util.near(m, rootNode, new double[] { 0.25, 0.25 }, 0.1, (value, dist) -> {
            nearList.add(value);
            distList.add(dist);
        });
        assertEquals(0, nearList.size());

        KDNearNode<String>  nearest = Util.nearest(m, rootNode, new double[] { 0.25, 0.25 });
        assertEquals("child3", nearest._nearest);
        assertEquals(0.25, nearest._dist, 0.001);
    }

    @Test
    void treeTest4() {
        // are these asserts actually valid?
        KDModel m = new MyKDModel();
        double[] init = new double[] { 0, 0 };
        KDNode<String> rootNode = new KDNode<String>(init, "root");

        Util.insert(m, rootNode, new double[] { 0.5, 0.5 }, "child1");
        Util.insert(m, rootNode, new double[] { 0.5, 075 }, "child2");

        // if the other view does the insert, the child is still found
        Util.insert(m, rootNode, new double[] { 0.5, 0.25 }, "child3");

        List<String> nearList = new ArrayList<String>();
        List<Double> distList = new ArrayList<Double>();
        // now it should find it
        Util.near(m, rootNode, new double[] { 0.25, 0.25 }, 0.26, (value, dist) -> {
            nearList.add(value);
            distList.add(dist);
        });
        assertEquals(1, nearList.size());
        assertEquals("child3", nearList.get(0));
        assertEquals(0.25, distList.get(0), 0.001);

        // and it should still find it
        KDNearNode<String>  nearest = Util.nearest(m, rootNode, new double[] { 0.25, 0.25 });
        assertEquals("child3", nearest._nearest);
        assertEquals(0.25, nearest._dist, 0.001);
    }

    @Test
    void treeTest5() {
        // are these asserts actually valid?
        KDModel m = new MyKDModel();
        double[] init = new double[] { 0, 0 };
        KDNode<String> rootNode = new KDNode<String>(init, "root");

        Util.insert(m, rootNode, new double[] { 0.5, 0.5 }, "child1");
        Util.insert(m, rootNode, new double[] { 0.5, 075 }, "child2");

        List<String> nearList = new ArrayList<String>();
        List<Double> distList = new ArrayList<Double>();
        // now it should find it
        Util.near(m, rootNode, new double[] { 0.25, 0.25 }, 0.26, (value, dist) -> {
            nearList.add(value);
            distList.add(dist);
        });
        assertEquals(0, nearList.size());

        // do the insert *after* it would have been found...
        Util.insert(m, rootNode, new double[] { 0.5, 0.25 }, "child3");

        // and it should still find it
        KDNearNode<String>  nearest = Util.nearest(m, rootNode, new double[] { 0.25, 0.25 });
        assertEquals("child3", nearest._nearest);
        assertEquals(0.25, nearest._dist, 0.001);
    }
}
