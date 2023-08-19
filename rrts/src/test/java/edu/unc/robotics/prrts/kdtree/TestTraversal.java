package edu.unc.robotics.prrts.kdtree;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.List;

import org.junit.jupiter.api.Test;

public class TestTraversal {

    KDModel myModel = new KDModel() {

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
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'dist'");
        }

        @Override
        public void steer(double[] nearConfig, double[] newConfig, double dist) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'steer'");
        }

    };

    @Test
    void testInsert() {
        KDNode<String> n = new KDNode<String>(new double[] { 0, 0 }, "n");
        Util.insert(myModel, n, new double[] { 0.5, 0 }, "one");
        List<String> s = Util.values(n);
        assertEquals(2, s.size());
        KDNode<String> a = n.getA();
        KDNode<String> b = n.getB();
        assertNull(a);
        assertEquals("one", b.getValue());

    }

}
