package org.team100.lib.rrt;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNearNode;
import org.team100.lib.index.KDNode;
import org.team100.lib.index.KDTree;
import org.team100.lib.rrt.example.full_state_arena.FullStateHolonomicArena;
import org.team100.lib.space.Sample;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N4;

public class TestRRTStar7 {

    @Test
    void testGoalRight() {
        assertTrue(RRTStar7.goalRight(0, 0, 0.5, 1.0, 2.0));
        assertTrue(RRTStar7.goalRight(0, 1, 5, 1, 2.0));
        assertTrue(RRTStar7.goalRight(0, 1, 0, -2, 2.0));
        assertFalse(RRTStar7.goalRight(0, 1, 0, 0, 2.0));
    }

    @Test
    void testTime() {
        assertTrue(RRTStar7.goalRight(0, 0, 0.5, 1.0, 2));

        assertEquals(0.724, RRTStar7.tSwitch(0, 0, 0.5, 1.0, 2), 0.001);

        assertEquals(1.0, RRTStar7.slowU(0, 0, 0.5, 1.0, 1.0), 0.001);

        // try sweeping the possibilities

        assertEquals(0.414, RRTStar7.tSwitch(0, 1, 0.5, 1, 2), 0.001);
        assertEquals(0.732, RRTStar7.tSwitch(0, 1, 1, 1, 2), 0.001);
        assertEquals(1.000, RRTStar7.tSwitch(0, 1, 1.5, 1, 2), 0.001);

        assertEquals(0.449, RRTStar7.tSwitch(0, 2, 1, 2, 2), 0.001);
        assertEquals(1.162, RRTStar7.tSwitch(0, 2, 3, 2, 2), 0.001);
        assertEquals(1.742, RRTStar7.tSwitch(0, 2, 5, 2, 2), 0.001);

        // assertEquals(1.0, RRTStar7.slowU(0, 2, 3, 2, 1.2), 0.001);
    }

    @Test
    void testIntercepts() {
        assertEquals(0, RRTStar7.c_minus(0, 0, 1), 0.001);
        assertEquals(0, RRTStar7.c_plus(0, 0, 1), 0.001);

        assertEquals(0.5, RRTStar7.c_minus(0, 1, 1), 0.001);
        assertEquals(-0.5, RRTStar7.c_plus(0, 1, 1), 0.001);

        assertEquals(1.5, RRTStar7.c_minus(1, 1, 1), 0.001);
        assertEquals(0.5, RRTStar7.c_plus(1, 1, 1), 0.001);

        assertEquals(-0.5, RRTStar7.c_minus(-1, 1, 1), 0.001);
        assertEquals(-1.5, RRTStar7.c_plus(-1, 1, 1), 0.001);

        assertEquals(0.5, RRTStar7.c_minus(0, -1, 1), 0.001);
        assertEquals(-0.5, RRTStar7.c_plus(0, -1, 1), 0.001);

        assertEquals(2, RRTStar7.c_minus(0, 2, 1), 0.001);
        assertEquals(-2, RRTStar7.c_plus(0, 2, 1), 0.001);

        assertEquals(0.25, RRTStar7.c_minus(0, 1, 2), 0.001);
        assertEquals(-0.25, RRTStar7.c_plus(0, 1, 2), 0.001);

        // these are cases for the switching point test below
        // these curves don't intersect at all
        assertEquals(-1, RRTStar7.c_minus(-3, 2, 1), 0.001);
        assertEquals(0, RRTStar7.c_plus(2, 2, 1), 0.001);

        // these curves intersect exactly once at the origin
        assertEquals(0, RRTStar7.c_minus(-2, 2, 1), 0.001);
        assertEquals(0, RRTStar7.c_plus(2, 2, 1), 0.001);

        // these two curves intersect twice, once at (0.5,1) and once at (0.5,-1)
        assertEquals(1, RRTStar7.c_minus(-1, 2, 1), 0.001);
        assertEquals(0, RRTStar7.c_plus(2, 2, 1), 0.001);
    }

    // TODO: also exercise the longer paths that cross the origin
    @Test
    void testQSwitch() {
        assertEquals(-0.5, RRTStar7.qSwitchIplusGminus(-3, 2, 2, 2, 1), 0.001);
        assertEquals(0, RRTStar7.qSwitchIplusGminus(-2, 2, 2, 2, 1), 0.001);
        assertEquals(0.5, RRTStar7.qSwitchIplusGminus(-1, 2, 2, 2, 1), 0.001);

        assertEquals(-0.5, RRTStar7.qSwitchIminusGplus(2, -2, -3, -2, 1), 0.001);
        assertEquals(0.0, RRTStar7.qSwitchIminusGplus(2, -2, -2, -2, 1), 0.001);
        assertEquals(0.5, RRTStar7.qSwitchIminusGplus(2, -2, -1, -2, 1), 0.001);
    }

    // TODO: also exercise the longer paths that cross the origin
    @Test
    void testQDotSwitch() {
        assertEquals(3.000, RRTStar7.qDotSwitchIplusGminus(-3, 2, 2, 2, 1), 0.001);
        assertEquals(2.828, RRTStar7.qDotSwitchIplusGminus(-2, 2, 2, 2, 1), 0.001);
        assertEquals(2.645, RRTStar7.qDotSwitchIplusGminus(-1, 2, 2, 2, 1), 0.001);

        assertEquals(-3.0, RRTStar7.qDotSwitchIminusGplus(2, -2, -3, -2, 1), 0.001);
        assertEquals(-2.828, RRTStar7.qDotSwitchIminusGplus(2, -2, -2, -2, 1), 0.001);
        assertEquals(-2.645, RRTStar7.qDotSwitchIminusGplus(2, -2, -1, -2, 1), 0.001);
    }

    @Test
    void testTime2() {
        // no movement == zero time
        assertFalse(RRTStar7.goalRight(0, 0, 0, 0, 2.5));
        assertEquals(0, RRTStar7.tSwitch(0, 0, 0, 0, 2.5), 0.001);
        // simply move back, rest-to-rest
        assertFalse(RRTStar7.goalRight(1, 0, 0, 0, 2.5));
        assertEquals(1.264, RRTStar7.tSwitch(1, 0, 0, 0, 2.5), 0.001);
        // slows to a stop, back, stop, forward, motion-to-motion
        // this is wrong.
        assertFalse(RRTStar7.goalRight(1, 1, -1, 1, 2.5));
        assertEquals(1.159, RRTStar7.tSwitch(1, 1, -1, 1, 2.5), 0.001);
    }

    @Test
    void testNearest() {
        /**
         * _init = { 15.5, 0, 6.75, 0 });
         * _goal = { 1.93, 0, 2.748, 0 });
         */
        final FullStateHolonomicArena arena = new FullStateHolonomicArena();
        KDNode<Node<N4>> T_a = new KDNode<>(new Node<>(arena.initial()));
        KDNode<Node<N4>> T_b = new KDNode<>(new Node<>(arena.goal()));
        final RRTStar7<FullStateHolonomicArena> solver = new RRTStar7<>(arena, new Sample<>(arena), 3, T_a, T_b);

        // add a node
        KDTree.insert(arena, T_a, new Node<>(new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 0, 0, 0, 0 })));
        // look for it
        KDNearNode<Node<N4>> near = solver.Nearest(new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1, 0, 0, 0 }), T_a,
                true);
        // find it: note this the time, not the Euclidean distance
        assertEquals(1.797, near._dist, 0.001);
        assertArrayEquals(new double[] { 0, 0, 0, 0 }, near._nearest.getState().getData(), 0.001);
    }

    @Test
    void testNearest2() {
        /**
         * _init = { 15.5, 0, 6.75, 0 });
         * _goal = { 1.93, 0, 2.748, 0 });
         */
        final FullStateHolonomicArena arena = new FullStateHolonomicArena();
        KDNode<Node<N4>> T_a = new KDNode<>(new Node<>(arena.initial()));
        KDNode<Node<N4>> T_b = new KDNode<>(new Node<>(arena.goal()));
        final RRTStar7<FullStateHolonomicArena> solver = new RRTStar7<>(arena, new Sample<>(arena), 3, T_a, T_b);

        KDTree.insert(arena, T_a, new Node<>(new Matrix<>(Nat.N4(), Nat.N1(), new double[] { -1, 1, 0, 0 })));
        KDNearNode<Node<N4>> near = solver.Nearest(new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1, 1, 0, 0 }), T_a,
                true);
        assertEquals(1.798, near._dist, 0.001);
        assertArrayEquals(new double[] { -1, 1, 0, 0 }, near._nearest.getState().getData(), 0.001);
    }

}
