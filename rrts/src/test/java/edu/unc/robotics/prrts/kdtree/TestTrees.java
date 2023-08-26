package edu.unc.robotics.prrts.kdtree;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDModel;
import org.team100.lib.index.KDNearNode;
import org.team100.lib.index.KDNode;
import org.team100.lib.index.KDTree;
import org.team100.lib.space.Point;


public class TestTrees {
    private static class MyKDModel implements KDModel {
        @Override
        public int dimensions() {
            return 2;
        }

        @Override
        public double[] getMin() {
            return new double[] { 0, 0 };
        }

        @Override
        public double[] getMax() {
            return new double[] { 1, 1 };
        }

        @Override
        public double dist(double[] start, double[] end) {
            return Math.sqrt(Math.pow(start[0] - end[0], 2) + Math.pow(start[1] - end[1], 2));
        }

        @Override
        public double[] steer(KDNearNode<Node> x_nearest, double[] newConfig) {
            throw new UnsupportedOperationException("Unimplemented method 'steer'");
        }

        @Override
        public void setStepNo(int stepNo) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void setRadius(double radius) {
            // TODO Auto-generated method stub
            
        }
    }

    static class StringPoint implements Point {
        private final String v;
        private final double[] _config;

        public StringPoint(String v, double[] _config) {
            this.v = v;
            this._config = _config;
        }

        @Override
        public double[] getState() {
            return _config;
        }

        public String get_v() {
            return v;

        }
    }

    @Test
    void treeTest() {
        KDModel m = new MyKDModel();
        double[] init = new double[] { 0, 0 };
        KDNode<StringPoint> rootNode = new KDNode<>(new StringPoint("root", init));
        KDTree.insert(m, rootNode,  new StringPoint("child1", new double[] { 0.5, 0.5 }));
        KDTree.insert(m, rootNode,  new StringPoint("child2", new double[] { 0.5, 0.75 }));
        KDTree.insert(m, rootNode,  new StringPoint("child3", new double[] { 0.5, 0.25 }));

        List<StringPoint> nearList = new ArrayList<>();
        List<Double> distList = new ArrayList<>();
        KDTree.near(m, rootNode, new double[] { 0.25, 0.25 }, 0.5, (value, dist) -> {
            nearList.add(value);
            distList.add(dist);
        });
        assertEquals(3, nearList.size());
        assertEquals("root", nearList.get(0).get_v());
        assertEquals("child1", nearList.get(1).get_v());
        assertEquals("child3", nearList.get(2).get_v());
        assertEquals(0.353, distList.get(0), 0.001);
        assertEquals(0.353, distList.get(1), 0.001);
        assertEquals(0.25, distList.get(2), 0.001);
    }

    @Test
    void treeTest2() {
        KDModel m = new MyKDModel();
        double[] init = new double[] { 0, 0 };
        KDNode<StringPoint> rootNode = new KDNode<>(new StringPoint("root", init));
        KDTree.insert(m, rootNode, new StringPoint("child1", new double[] { 0.5, 0.5 }));
        KDTree.insert(m, rootNode, new StringPoint("child2", new double[] { 0.5, 0.75 }));
        KDTree.insert(m, rootNode,  new StringPoint("child3", new double[] { 0.5, 0.25 }));

        List<StringPoint> nearList = new ArrayList<>();
        List<Double> distList = new ArrayList<>();
        // small radius
        KDTree.near(m, rootNode, new double[] { 0.25, 0.25 }, 0.1, (value, dist) -> {
            nearList.add(value);
            distList.add(dist);
        });
        assertEquals(0, nearList.size());

        KDNearNode<StringPoint> nearest = KDTree.nearest(m, rootNode, new double[] { 0.25, 0.25 });
        assertEquals("child3", nearest._nearest.get_v());
        assertEquals(0.25, nearest._dist, 0.001);
    }

    @Test
    void treeTest3() {
        // are these asserts actually valid?
        KDModel m = new MyKDModel();
        double[] init = new double[] { 0, 0 };
        KDNode<StringPoint> rootNode = new KDNode<>(new StringPoint("root", init));

        KDTree.insert(m, rootNode, new StringPoint("child1", new double[] { 0.5, 0.5 }));
        KDTree.insert(m, rootNode,  new StringPoint("child2", new double[] { 0.5, 0.75 }));

        // if the other view does the insert, the child is still found
        KDTree.insert(m, rootNode,  new StringPoint("child3", new double[] { 0.5, 0.25 }));

        List<StringPoint> nearList = new ArrayList<>();
        List<Double> distList = new ArrayList<>();
        // small radius
        KDTree.near(m, rootNode, new double[] { 0.25, 0.25 }, 0.1, (value, dist) -> {
            nearList.add(value);
            distList.add(dist);
        });
        assertEquals(0, nearList.size());

        KDNearNode<StringPoint> nearest = KDTree.nearest(m, rootNode, new double[] { 0.25, 0.25 });
        assertEquals("child3", nearest._nearest.get_v());
        assertEquals(0.25, nearest._dist, 0.001);
    }

    @Test
    void treeTest4() {
        // are these asserts actually valid?
        KDModel m = new MyKDModel();
        double[] init = new double[] { 0, 0 };
        KDNode<StringPoint> rootNode = new KDNode<>(new StringPoint("root", init));

        KDTree.insert(m, rootNode, new StringPoint("child1", new double[] { 0.5, 0.5 }));
        KDTree.insert(m, rootNode,  new StringPoint("child2", new double[] { 0.5, 075 }));

        // if the other view does the insert, the child is still found
        KDTree.insert(m, rootNode,  new StringPoint("child3", new double[] { 0.5, 0.25 }));

        List<StringPoint> nearList = new ArrayList<>();
        List<Double> distList = new ArrayList<>();
        // now it should find it
        KDTree.near(m, rootNode, new double[] { 0.25, 0.25 }, 0.26, (value, dist) -> {
            nearList.add(value);
            distList.add(dist);
        });
        assertEquals(1, nearList.size());
        assertEquals("child3", nearList.get(0).get_v());
        assertEquals(0.25, distList.get(0), 0.001);

        // and it should still find it
        KDNearNode<StringPoint> nearest = KDTree.nearest(m, rootNode, new double[] { 0.25, 0.25 });
        assertEquals("child3", nearest._nearest.get_v());
        assertEquals(0.25, nearest._dist, 0.001);
    }

    @Test
    void treeTest5() {
        // are these asserts actually valid?
        KDModel m = new MyKDModel();
        double[] init = new double[] { 0, 0 };
        KDNode<StringPoint> rootNode = new KDNode<>(new StringPoint("root", init));

        KDTree.insert(m, rootNode,  new StringPoint("child1", new double[] { 0.5, 0.5 }));
        KDTree.insert(m, rootNode,  new StringPoint("child2", new double[] { 0.5, 075 }));

        List<StringPoint> nearList = new ArrayList<>();
        List<Double> distList = new ArrayList<>();
        // now it should find it
        KDTree.near(m, rootNode, new double[] { 0.25, 0.25 }, 0.26, (value, dist) -> {
            nearList.add(value);
            distList.add(dist);
        });
        assertEquals(0, nearList.size());

        // do the insert *after* it would have been found...
        KDTree.insert(m, rootNode, new StringPoint("child3", new double[] { 0.5, 0.25 }));

        // and it should still find it
        KDNearNode<StringPoint> nearest = KDTree.nearest(m, rootNode, new double[] { 0.25, 0.25 });
        assertEquals("child3", nearest._nearest.get_v());
        assertEquals(0.25, nearest._dist, 0.001);
    }
}
