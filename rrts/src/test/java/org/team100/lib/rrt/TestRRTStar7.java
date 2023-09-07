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

        // there are three solutions to this case; tSwitch returns the fastest.
        // see below for tLimit and tMirror.
        assertEquals(0.414, RRTStar7.tSwitch(0, 1, 0.5, 1, 2), 0.001);
        // the fast way
        assertEquals(0.732, RRTStar7.tSwitch(0, 1, 1, 1, 2), 0.001);
        assertEquals(1.000, RRTStar7.tSwitch(0, 1, 1.5, 1, 2), 0.001);

        // same story here, we choose the slow path but there's also a fast one.
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

    @Test
    void testQSwitch() {
        assertEquals(-0.5, RRTStar7.qSwitchIplusGminus(-3, 2, 2, 2, 1), 0.001);
        assertEquals(0, RRTStar7.qSwitchIplusGminus(-2, 2, 2, 2, 1), 0.001);
        assertEquals(0.5, RRTStar7.qSwitchIplusGminus(-1, 2, 2, 2, 1), 0.001);

        assertEquals(-0.5, RRTStar7.qSwitchIminusGplus(2, -2, -3, -2, 1), 0.001);
        assertEquals(0.0, RRTStar7.qSwitchIminusGplus(2, -2, -2, -2, 1), 0.001);
        assertEquals(0.5, RRTStar7.qSwitchIminusGplus(2, -2, -1, -2, 1), 0.001);

        // these are all a little different just to avoid zero as the answer
        assertEquals(0.5, RRTStar7.qSwitchIplusGminus(2, 2, -1, 2, 1), 0.001);
        assertEquals(0.5, RRTStar7.qSwitchIplusGminus(-1, 2, 2, -2, 1), 0.001);
        assertEquals(0.5, RRTStar7.qSwitchIminusGplus(2, 2, -1, 2, 1), 0.001);
        assertEquals(0.5, RRTStar7.qSwitchIminusGplus(-1, 2, 2, -2, 1), 0.001);
    }

    @Test
    void testQDotSwitch() {
        assertEquals(3.000, RRTStar7.qDotSwitchIplusGminus(-3, 2, 2, 2, 1), 0.001);
        assertEquals(2.828, RRTStar7.qDotSwitchIplusGminus(-2, 2, 2, 2, 1), 0.001);
        assertEquals(2.645, RRTStar7.qDotSwitchIplusGminus(-1, 2, 2, 2, 1), 0.001);

        assertEquals(-3.0, RRTStar7.qDotSwitchIminusGplus(2, -2, -3, -2, 1), 0.001);
        assertEquals(-2.828, RRTStar7.qDotSwitchIminusGplus(2, -2, -2, -2, 1), 0.001);
        assertEquals(-2.645, RRTStar7.qDotSwitchIminusGplus(2, -2, -1, -2, 1), 0.001);

        // from 2,2 to -2,2, I+G- is invalid
        assertEquals(0, RRTStar7.qDotSwitchIplusGminus(2, 2, -2, 2, 1), 0.001);
        // from -2,2 to 2,-2 switches in the same place as -2,2->2,2
        assertEquals(2.828, RRTStar7.qDotSwitchIplusGminus(-2, 2, 2, -2, 1), 0.001);
        // from 2,2 to -2,2 switches at the bottom
        assertEquals(-2.828, RRTStar7.qDotSwitchIminusGplus(2, 2, -2, 2, 1), 0.001);
        // from -2,2 to 2,-2, I-G+ is invalid
        assertEquals(0, RRTStar7.qDotSwitchIminusGplus(-2, 2, 2, -2, 1), 0.001);
    }

    @Test
    void testTSwitch() {
        // from -2 to 2, the 'fast' and normal way
        assertEquals(1.656, RRTStar7.tSwitchIplusGminus(-2, 2, 2, 2, 1), 0.001);
        // this path goes from (-2,2) to (0,0) and then to (2,2)
        assertEquals(4.000, RRTStar7.tSwitchIminusGplus(-2, 2, 2, 2, 1), 0.001);

        // the opposite order, 2 to -2, this is a completely invalid result,
        // traversing Iplus backwards, and then Gminus backwards.
        assertEquals(-4.000, RRTStar7.tSwitchIplusGminus(2, 2, -2, 2, 1), 0.001);
        // from 2 to -2 is the 'long way around' across the x-axis and back.
        assertEquals(9.656, RRTStar7.tSwitchIminusGplus(2, 2, -2, 2, 1), 0.001);

        // diagonal, the 'fast' and normal way
        assertEquals(5.656, RRTStar7.tSwitchIplusGminus(-2, 2, 2, -2, 1), 0.001);
        // this is completely invalid
        assertEquals(0, RRTStar7.tSwitchIminusGplus(-2, 2, 2, -2, 1), 0.001);
        // this is invalid, it should yield like NaN or something
        assertEquals(0, RRTStar7.tSwitchIplusGminus(2, -2, -2, 2, 1), 0.001);
        // from 2 to -2 is the 'long way around' across the x-axis and back.
        assertEquals(5.656, RRTStar7.tSwitchIminusGplus(2, -2, -2, 2, 1), 0.001);

        // another problem case
        // this can't be done
        assertEquals(Double.NaN, RRTStar7.tSwitchIminusGplus(-1, 1, 0, 0, 1), 0.001);
        // the "normal" way
        assertEquals(1.449, RRTStar7.tSwitchIplusGminus(-1, 1, 0, 0, 1), 0.001);

    }

    @Test
    void testTime2() {
        // no movement == zero time
        assertFalse(RRTStar7.goalRight(0, 0, 0, 0, 2.5));
        assertEquals(0, RRTStar7.tSwitch(0, 0, 0, 0, 2.5), 0.001);

        // simply move back, rest-to-rest
        assertFalse(RRTStar7.goalRight(1, 0, 0, 0, 2.5));

        // (1,0) -> (0,0) takes 1.264
        assertEquals(1.264, RRTStar7.tSwitch(1, 0, 0, 0, 2.5), 0.001);

        // slows to a stop, back, stop, forward, motion-to-motion
        assertFalse(RRTStar7.goalRight(1, 1, -1, 1, 2.5));
        assertEquals(2.759, RRTStar7.tSwitch(1, 1, -1, 1, 2.5), 0.001);

        // used below
        assertEquals(0.985, RRTStar7.tSwitch(-1, 1, 0, 0, 2.5), 0.001);

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

        solver.setRadius(10);

        // add a node
        KDTree.insert(arena, T_a, new Node<>(new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 0, 0, 0, 0 })));
        System.out.println(T_a);

        // look for it
        KDNearNode<Node<N4>> near = solver.BangBangNearest(
                new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1, 0, 0, 0 }), T_a,
                true);

        System.out.println(near);
        // find it: note this the time, not the Euclidean distance
        // (1,0) to (0,0) takes 1.264 (see above)
        assertEquals(1.264, near._dist, 0.001);
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
        solver.setRadius(10);

        KDTree.insert(arena, T_a, new Node<>(new Matrix<>(Nat.N4(), Nat.N1(), new double[] { -1, 1, 0, 0 })));
        KDNearNode<Node<N4>> near = solver.BangBangNearest(
                new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1, 1, 0, 0 }), T_a,
                true);
        // (-1,1) to (0,0)
        assertEquals(2.759, near._dist, 0.001);
        assertArrayEquals(new double[] { -1, 1, 0, 0 }, near._nearest.getState().getData(), 0.001);
    }

    @Test
    void testSteer() {
        // "steering" in this case just means "follow until obstacle"
        // so it's nothing like the other "steer"

    }

    @Test
    void testTLimit() {
        // tLimit does not exist when there is no intersection.
        // this is invalid
        assertEquals(-1.732, RRTStar7.qDotLimitIplusGminus(0, 1, 1, 1, 2), 0.001);
        // the slow path doesn't exist
        assertEquals(Double.NaN, RRTStar7.qDotLimitIminusGplus(0, 1, 1, 1, 2), 0.001);
        // so this should also not exist
        assertEquals(Double.NaN, RRTStar7.tLimit(0, 1, 1, 1, 2), 0.001);

        // tLimit exists when there are both I+G- and I-G+ solutions
        // (0,1) -> (0.5,1) I-G+ intersect at one point not two, so
        // tLimit and tMirror should be the same.
        // this is invalid, it's the fast path
        assertEquals(-1.414, RRTStar7.qDotLimitIplusGminus(0, 1, 0.5, 1, 2), 0.001);
        // the single intersection is on the x axis
        assertEquals(0.000, RRTStar7.qDotLimitIminusGplus(0, 1, 0.5, 1, 2), 0.001);
        // 0.5s to the axis, 0.5s back up to g
        assertEquals(1.000, RRTStar7.tLimit(0, 1, 0.5, 1, 2), 0.001);

        // (0,1) -> (0.25,1) has two intersections.
        // this is invalid for tLimit, it's the fast path
        assertEquals(-1.224, RRTStar7.qDotLimitIplusGminus(0, 1, 0.25, 1, 2), 0.001);
        // the slow path goes a little slower than the initial state
        assertEquals(0.707, RRTStar7.qDotLimitIminusGplus(0, 1, 0.25, 1, 2), 0.001);
        // the states are nearby, doesn't take long
        assertEquals(0.292, RRTStar7.tLimit(0, 1, 0.25, 1, 2), 0.001);
        // for comparison, this is tSwitch, a little faster
        assertEquals(0.224, RRTStar7.tSwitch(0, 1, 0.25, 1, 2), 0.001);

        // (0,2) -> (1,2)
        // I+G- yields the fast path
        assertEquals(-2.449, RRTStar7.qDotLimitIplusGminus(0, 2, 1, 2, 2), 0.001);
        // I-G+ is the slow path, slower than intial/goal speed
        assertEquals(1.414, RRTStar7.qDotLimitIminusGplus(0, 2, 1, 2, 2), 0.001);
        // time to traverse the slow path
        assertEquals(0.585, RRTStar7.tLimit(0, 2, 1, 2, 2), 0.001);
        // for comparison, this is tSwitch, a little faster.
        assertEquals(0.449, RRTStar7.tSwitch(0, 2, 1, 2, 2), 0.001);

        // there are no cases here with i and g on opposite sides
        // of the qdot=0 axis, because there's no "limit" or "mirror"
        // possible in that case
    }

    @Test
    void testTMirror() {
        // same cases as above.

        // should not exist
        assertEquals(Double.NaN, RRTStar7.tMirror(0, 1, 1, 1, 2), 0.001);

        // (0,1) -> (0.5,1) yields just one intersection
        // so tLimit and tMirror are the same:
        assertEquals(1.000, RRTStar7.tMirror(0, 1, 0.5, 1, 2), 0.001);

        // two intersections, recall tLimit was 0.292
        assertEquals(1.707, RRTStar7.tMirror(0, 1, 0.25, 1, 2), 0.001);

        // two intersections, recall that tLimit was 0.585
        assertEquals(3.414, RRTStar7.tMirror(0, 2, 1, 2, 2), 0.001);
    }

}
