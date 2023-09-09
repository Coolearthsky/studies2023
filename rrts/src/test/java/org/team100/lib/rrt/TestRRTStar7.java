package org.team100.lib.rrt;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNearNode;
import org.team100.lib.index.KDNode;
import org.team100.lib.index.KDTree;
import org.team100.lib.rrt.RRTStar7.Trajectory;
import org.team100.lib.rrt.RRTStar7.Trajectory.Axis;
import org.team100.lib.rrt.example.full_state_arena.FullStateHolonomicArena;
import org.team100.lib.space.Sample;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

public class TestRRTStar7 {

    @Test
    void testGoalRight() {
        assertTrue(RRTStar7.goalRight(0, 0, 0.5, 1.0, 2.0));
        assertTrue(RRTStar7.goalRight(0, 1, 5, 1, 2.0));
        assertTrue(RRTStar7.goalRight(0, 1, 0, -2, 2.0));
        assertFalse(RRTStar7.goalRight(0, 1, 0, 0, 2.0));
        // no movement == zero time
        assertFalse(RRTStar7.goalRight(0, 0, 0, 0, 2.5));
        // simply move back, rest-to-rest
        assertFalse(RRTStar7.goalRight(1, 0, 0, 0, 2.5));
        // slows to a stop, back, stop, forward, motion-to-motion
        assertFalse(RRTStar7.goalRight(1, 1, -1, 1, 2.5));
    }

    @Test
    void testSlowU() {
        System.out.println("faster = higher U, I+G-");
        Axis slowU = RRTStar7.slowU(0, 0, 0.5, 1.0, 0.5);
        assertEquals(4.828, slowU.s1.u, 0.001);
        assertEquals(0.353, slowU.s1.t, 0.001);
        assertEquals(-4.828, slowU.s2.u, 0.001);
        assertEquals(0.146, slowU.s2.t, 0.001);

        System.out.println("example from below, tswitch is 0.724s for U=2, I+G-");
        Axis slowU2 = RRTStar7.slowU(0, 0, 0.5, 1.0, 0.724);
        assertEquals(2.004, slowU2.s1.u, 0.001);
        assertEquals(0.611, slowU2.s1.t, 0.001);
        assertEquals(-2.004, slowU2.s2.u, 0.001);
        assertEquals(0.112, slowU2.s2.t, 0.001);

        System.out.println("just faster than no switch, I+G-");
        Axis slowU3 = RRTStar7.slowU(0, 0, 0.5, 1.0, 0.9);
        assertEquals(1.241, slowU3.s1.u, 0.001);
        assertEquals(0.852, slowU3.s1.t, 0.001);
        assertEquals(-1.241, slowU3.s2.u, 0.001);
        assertEquals(0.047, slowU3.s2.t, 0.001); // very short

        System.out.println("no switch I+G-");
        Axis slowU4 = RRTStar7.slowU(0, 0, 0.5, 1.0, 1);
        assertEquals(1.000, slowU4.s1.u, 0.001);
        assertEquals(1.000, slowU4.s1.t, 0.001);
        assertEquals(-1.000, slowU4.s2.u, 0.001);
        assertEquals(0, slowU4.s2.t, 0.001); // zero!

        System.out.println("just slower than no switch I-G+");
        Axis slowU5 = RRTStar7.slowU(0, 0, 0.5, 1.0, 1.1);
        assertEquals(-0.995, slowU5.s1.u, 0.001);
        assertEquals(1.052, slowU5.s1.t, 0.001);
        assertEquals(0.995, slowU5.s2.u, 0.001);
        assertEquals(0.047, slowU5.s2.t, 0.001);

        System.out.println("much slower I-G+");
        Axis slowU6 = RRTStar7.slowU(0, 0, 0.5, 1.0, 2);
        assertEquals(-0.809, slowU6.s1.u, 0.001);
        assertEquals(1.618, slowU6.s1.t, 0.001);
        assertEquals(0.809, slowU6.s2.u, 0.001);
        assertEquals(0.381, slowU6.s2.t, 0.001);

        System.out.println("reflect the above case, I+G-");
        Axis slowU7 = RRTStar7.slowU(0, 0, -0.5, -1.0, 2);
        assertEquals(0.809, slowU7.s1.u, 0.001);
        assertEquals(0.381, slowU7.s1.t, 0.001);
        assertEquals(-0.809, slowU7.s2.u, 0.001);
        assertEquals(1.618, slowU7.s2.t, 0.001);

        // single intersection at u=2
        assertEquals(0.414, RRTStar7.tSwitch(0, 1, 0.5, 1, 2), 0.001);
        assertEquals(1.000, RRTStar7.tLimit(0, 1, 0.5, 1, 2), 0.001);
        assertEquals(1.000, RRTStar7.tMirror(0, 1, 0.5, 1, 2), 0.001);
        // finds the tswitch u=2 solution, I+G-
        Axis slowU8 = RRTStar7.slowU(0, 1, 0.5, 1, 0.414);
        assertEquals(2.007, slowU8.s1.u, 0.001);
        assertEquals(0.207, slowU8.s1.t, 0.001);
        assertEquals(-2.007, slowU8.s2.u, 0.001);
        assertEquals(0.207, slowU8.s2.t, 0.001); // midpoint
        // finds the tlimit u=2 solution, I-G+
        Axis slowU9 = RRTStar7.slowU(0, 1, 0.5, 1, 1.000);
        assertEquals(-2.0, slowU9.s1.u, 0.001);
        assertEquals(0.5, slowU9.s1.t, 0.001);
        assertEquals(2.0, slowU9.s2.u, 0.001);
        assertEquals(0.5, slowU9.s2.t, 0.001); // midpoint
        // this is a loop below the axis, I-G+
        Axis slowU10 = RRTStar7.slowU(0, 1, 0.5, 1, 2.000);
        assertEquals(-1.5, slowU10.s1.u, 0.001);
        assertEquals(1.0, slowU10.s1.t, 0.001);
        assertEquals(1.5, slowU10.s2.u, 0.001);
        assertEquals(1.0, slowU10.s2.t, 0.001); // midpoint

        // no limit or mirror at u=2
        assertEquals(0.732, RRTStar7.tSwitch(0, 1, 1, 1, 2), 0.001);
        assertEquals(Double.NaN, RRTStar7.tLimit(0, 1, 1, 1, 2), 0.001);
        assertEquals(Double.NaN, RRTStar7.tMirror(0, 1, 1, 1, 2), 0.001);
        // discover the tswitch solution
        Axis slowU11 = RRTStar7.slowU(0, 1, 1, 1, 0.732);
        assertEquals(2.000, slowU11.s1.u, 0.001);
        assertEquals(0.366, slowU11.s1.t, 0.001);
        assertEquals(-2.000, slowU11.s2.u, 0.001);
        assertEquals(0.366, slowU11.s2.t, 0.001); // midpoint
        System.out.println("the correct answer is to just drift, u=0");
        Axis slowU12 = RRTStar7.slowU(0, 1, 1, 1, 1.000);
        assertEquals(0, slowU12.s1.u, 0.001);
        assertEquals(1.000, slowU12.s1.t, 0.001);
        assertEquals(0, slowU12.s2.u, 0.001);
        assertEquals(0, slowU12.s2.t, 0.001); // no second segment
        // down to the axis and back, I-G+
        Axis slowU13 = RRTStar7.slowU(0, 1, 1, 1, 2.000);
        assertEquals(-1, slowU13.s1.u, 0.001);
        assertEquals(1, slowU13.s1.t, 0.001);
        assertEquals(1, slowU13.s2.u, 0.001);
        assertEquals(1, slowU13.s2.t, 0.001);

        // no limit or mirror at u=2
        assertEquals(1.000, RRTStar7.tSwitch(0, 1, 1.5, 1, 2), 0.001);
        assertEquals(Double.NaN, RRTStar7.tLimit(0, 1, 1.5, 1, 2), 0.001);
        assertEquals(Double.NaN, RRTStar7.tMirror(0, 1, 1.5, 1, 2), 0.001);
        // discover tswitch, I+G-
        Axis slowU14 = RRTStar7.slowU(0, 1, 1.5, 1, 1.000);
        assertEquals(2.000, slowU14.s1.u, 0.001);
        assertEquals(0.5, slowU14.s1.t, 0.001);
        assertEquals(-2.000, slowU14.s2.u, 0.001);
        assertEquals(0.5, slowU14.s2.t, 0.001); // midpoint
        // t=2, u=0.5, intersects at (0.75,0.5), I-G+
        Axis slowU15 = RRTStar7.slowU(0, 1, 1.5, 1, 2.000);
        assertEquals(-0.500, slowU15.s1.u, 0.001);
        assertEquals(1.0, slowU15.s1.t, 0.001);
        assertEquals(0.500, slowU15.s2.u, 0.001);
        assertEquals(1.0, slowU15.s2.t, 0.001); // midpoint
        // taking longer takes *more* u in order to slow down harder, I-G+
        Axis slowU16 = RRTStar7.slowU(0, 1, 1.5, 1, 3.000);
        assertEquals(-0.666, slowU16.s1.u, 0.001);
        assertEquals(1.5, slowU16.s1.t, 0.001);
        assertEquals(0.666, slowU16.s2.u, 0.001);
        assertEquals(1.5, slowU16.s2.t, 0.001); // midpoint

        // a short path with limit and mirror
        assertEquals(0.449, RRTStar7.tSwitch(0, 2, 1, 2, 2), 0.001);
        assertEquals(0.585, RRTStar7.tLimit(0, 2, 1, 2, 2), 0.001);
        assertEquals(3.414, RRTStar7.tMirror(0, 2, 1, 2, 2), 0.001);
        // discover tswitch (the fast path) I+G-
        Axis slowU17 = RRTStar7.slowU(0, 2, 1, 2, 0.449);
        assertEquals(2.023, slowU17.s1.u, 0.001);
        assertEquals(0.224, slowU17.s1.t, 0.001);
        assertEquals(-2.023, slowU17.s2.u, 0.001);
        assertEquals(0.224, slowU17.s2.t, 0.001); // midpoint
        // discover tlimit I-G+
        Axis slowU18 = RRTStar7.slowU(0, 2, 1, 2, 0.585);
        assertEquals(-1.986, slowU18.s1.u, 0.001);
        assertEquals(0.292, slowU18.s1.t, 0.001);
        assertEquals(1.986, slowU18.s2.u, 0.001);
        assertEquals(0.292, slowU18.s2.t, 0.001); // midpoint
        // discover tmirror I-G+
        Axis slowU19 = RRTStar7.slowU(0, 2, 1, 2, 3.414);
        assertEquals(-2.000, slowU19.s1.u, 0.001);
        assertEquals(1.707, slowU19.s1.t, 0.001);
        assertEquals(2.000, slowU19.s2.u, 0.001);
        assertEquals(1.707, slowU19.s2.t, 0.001); // midpoint
        // try a couple of times between tlimit and tmirror
        // it's possible you just need more u. I-G+
        Axis slowU20 = RRTStar7.slowU(0, 2, 1, 2, 1);
        assertEquals(-4.000, slowU20.s1.u, 0.001);
        assertEquals(0.5, slowU20.s1.t, 0.001);
        assertEquals(4.000, slowU20.s2.u, 0.001);
        assertEquals(0.5, slowU20.s2.t, 0.001); // midpoint
        // I-G+
        Axis slowU21 = RRTStar7.slowU(0, 2, 1, 2, 2);
        assertEquals(-3.000, slowU21.s1.u, 0.001);
        assertEquals(1.0, slowU21.s1.t, 0.001);
        assertEquals(3.000, slowU21.s2.u, 0.001);
        assertEquals(1.0, slowU21.s2.t, 0.001);

        // up from zero I+G-
        assertEquals(1, RRTStar7.tSwitch(1, 0, 2, 2, 2), 0.001);
        Axis slowU22 = RRTStar7.slowU(1, 0, 2, 2, 1);
        assertEquals(2.0, slowU22.s1.u, 0.001);
        assertEquals(1.0, slowU22.s1.t, 0.001);
        assertEquals(-2.0, slowU22.s2.u, 0.001);
        assertEquals(0.0, slowU22.s2.t, 0.001); // no second segment

        // from negative idot, cross the axis, switch. I+G-
        assertEquals(2.645, RRTStar7.tSwitch(0, -1, 3, 1, 2), 0.001);
        // discover tswitch
        Axis slowU23 = RRTStar7.slowU(0, -1, 3, 1, 2.645);
        assertEquals(2.000, slowU23.s1.u, 0.001);
        assertEquals(1.822, slowU23.s1.t, 0.001);
        assertEquals(-2.000, slowU23.s2.u, 0.001);
        assertEquals(0.822, slowU23.s2.t, 0.001);
    }

    @Test
    void testTSwitch() {

        assertEquals(0.724, RRTStar7.tSwitch(0, 0, 0.5, 1.0, 2), 0.001);

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

        // no movement == zero time
        assertEquals(0, RRTStar7.tSwitch(0, 0, 0, 0, 2.5), 0.001);

        // simply move back, rest-to-rest
        // (1,0) -> (0,0) takes 1.264
        assertEquals(1.264, RRTStar7.tSwitch(1, 0, 0, 0, 2.5), 0.001);

        // slows to a stop, back, stop, forward, motion-to-motion
        assertEquals(2.759, RRTStar7.tSwitch(1, 1, -1, 1, 2.5), 0.001);

        // (-1,1) -> (0,0), used below
        assertEquals(0.985, RRTStar7.tSwitch(-1, 1, 0, 0, 2.5), 0.001);

        // (0,0) -> (1,0), used below
        assertEquals(1.264, RRTStar7.tSwitch(0, 0, 1, 0, 2.5), 0.001);
        assertEquals(Double.NaN, RRTStar7.tLimit(0, 0, 1, 0, 2.5), 0.001);

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
        assertEquals(0.375, RRTStar7.qSwitchIplusGminus(0, 0, 0.5, 1.0, 2), 0.001);
        assertEquals(0.125, RRTStar7.qSwitchIminusGplus(0, 0, 0.5, 1.0, 2), 0.001);

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
        assertEquals(1.224, RRTStar7.qDotSwitchIplusGminus(0, 0, 0.5, 1.0, 2), 0.001);
        assertEquals(Double.NaN, RRTStar7.qDotSwitchIminusGplus(0, 0, 0.5, 1.0, 2), 0.001);

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
    void testTSwitchByPath() {
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
    void testNearest3() {
        /**
         * _init = { 15.5, 0, 6.75, 0 });
         * _goal = { 1.93, 0, 2.748, 0 });
         */
        final FullStateHolonomicArena arena = new FullStateHolonomicArena();
        KDNode<Node<N4>> T_a = new KDNode<>(new Node<>(arena.initial()));
        KDNode<Node<N4>> T_b = new KDNode<>(new Node<>(arena.goal()));
        final RRTStar7<FullStateHolonomicArena> solver = new RRTStar7<>(arena, new Sample<>(arena), 3, T_a, T_b);

        // note small radius; this won't find anything
        solver.setRadius(1);

        System.out.println(T_a);

        KDNearNode<Node<N4>> near = solver.BangBangNearest(
                new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1, 0, 0, 0 }), T_a,
                true);
        // this should find nothing
        assertNull(near);
    }

    static Matrix<N4, N1> s(double x1, double x2, double x3, double x4) {
        return new Matrix<>(Nat.N4(), Nat.N1(), new double[] { x1, x2, x3, x4 });
    }

    @Test
    void testOptimalTrajectory() {
        // same cases as below.
        // symmetrical, diagonal
        Trajectory t = RRTStar7.optimalTrajectory(s(0, 0, 0, 0), s(1, 0, 1, 0), 2.5);
        assertEquals(2.5, t.x.s1.u, 0.001);
        assertEquals(0.632, t.x.s1.t, 0.001);
        assertEquals(-2.5, t.x.s2.u, 0.001);
        assertEquals(0.632, t.x.s2.t, 0.001);
        assertEquals(2.5, t.y.s1.u, 0.001);
        assertEquals(0.632, t.y.s1.t, 0.001);
        assertEquals(-2.5, t.y.s2.u, 0.001);
        assertEquals(0.632, t.y.s2.t, 0.001);

        // x needs to be slowed at part-throttle while y goes full
        // both switch at the same time, in the middle.
        // this makes an S shape.
        t = RRTStar7.optimalTrajectory(s(0, 1, 0, 0), s(0.5, 1, 1, 0), 2.5);
        assertEquals(-1.912, t.x.s1.u, 0.001);
        assertEquals(0.632, t.x.s1.t, 0.001);
        assertEquals(1.912, t.x.s2.u, 0.001);
        assertEquals(0.632, t.x.s2.t, 0.001);
        assertEquals(2.5, t.y.s1.u, 0.001);
        assertEquals(0.632, t.y.s1.t, 0.001);
        assertEquals(-2.5, t.y.s2.u, 0.001);
        assertEquals(0.632, t.y.s2.t, 0.001);

        // x states are very close together
        // y is the same as above
        // the y fast path is in the x gap, so y goes a little slower than optimal
        // (without braking) to match x mirror.
        // this makes a more exaggerated S shape
        t = RRTStar7.optimalTrajectory(s(0, 1, 0, 0), s(0.25, 1, 1, 0), 2.5);
        assertEquals(-2.5, t.x.s1.u, 0.001);
        assertEquals(0.644, t.x.s1.t, 0.001);
        assertEquals(2.5, t.x.s2.u, 0.001);
        assertEquals(0.644, t.x.s2.t, 0.001);
        assertEquals(2.404, t.y.s1.u, 0.001);
        assertEquals(0.644, t.y.s1.t, 0.001);
        assertEquals(-2.404, t.y.s2.u, 0.001);
        assertEquals(0.644, t.y.s2.t, 0.001);

        // both x and y states are very close together.
        // as above the y fast path is in the x gap
        // but the y points are so close together that y needs to brake to match x
        // mirror.
        t = RRTStar7.optimalTrajectory(s(0, 1, 0, 1), s(0.25, 1, 0.5, 1), 2.5);
        assertEquals(-2.5, t.x.s1.u, 0.001);
        assertEquals(0.644, t.x.s1.t, 0.001);
        assertEquals(2.5, t.x.s2.u, 0.001);
        assertEquals(0.644, t.x.s2.t, 0.001);
        assertEquals(-1.898, t.y.s1.u, 0.001);
        assertEquals(0.644, t.y.s1.t, 0.001);
        assertEquals(1.898, t.y.s2.u, 0.001);
        assertEquals(0.644, t.y.s2.t, 0.001);

        // this shows that a diagonal not at 45 degrees involves u values
        // proportional to the angle.
        // it's just a straight line that speeds up in the middle.
        t = RRTStar7.optimalTrajectory(s(0, 2, 0, 1), s(1, 2, 0.5, 1), 2.5);
        assertEquals(2.5, t.x.s1.u, 0.001);
        assertEquals(0.219, t.x.s1.t, 0.001);
        assertEquals(-2.5, t.x.s2.u, 0.001);
        assertEquals(0.219, t.x.s2.t, 0.001);
        assertEquals(1.25, t.y.s1.u, 0.001);
        assertEquals(0.219, t.y.s1.t, 0.001);
        assertEquals(-1.25, t.y.s2.u, 0.001);
        assertEquals(0.219, t.y.s2.t, 0.001);

        // here's a longer-range case.  long-range cases are easier.
        // ahead x, go to a far-away spot and stop.
        // since there's initial velocity, the braking phase is longer.
        // this is also an example where the switching points
        // of the axes are not the same.
        // TODO: velocity limit for these cases
        t = RRTStar7.optimalTrajectory(s(0, 5, 0, 0), s(10, 0, 5, 0), 2.5);
        assertEquals(2.5, t.x.s1.u, 0.001);
        assertEquals(0.449, t.x.s1.t, 0.001);
        assertEquals(-2.5, t.x.s2.u, 0.001);
        assertEquals(2.449, t.x.s2.t, 0.001);
        assertEquals(2.379, t.y.s1.u, 0.001);
        assertEquals(1.449, t.y.s1.t, 0.001);
        assertEquals(-2.379, t.y.s2.u, 0.001);
        assertEquals(1.449, t.y.s2.t, 0.001);
    }

    @Test
    void testTOptimal() {
        // no movement = no time

        // same trajectory in both axes
        // x: (0,0) -> (1,0)
        // y: (0,0) -> (1,0)
        assertEquals(1.264, RRTStar7.tOptimal(s(0, 0, 0, 0), s(1, 0, 1, 0), 2.5), 0.001);

        // slow one is slower than tmirror
        // x: (0,1) -> (0.5,1): tswitch = 0.414, tlimit=tmirror=1
        // y: (0,0) -> (1,0)
        assertEquals(1.264, RRTStar7.tOptimal(s(0, 1, 0, 0), s(0.5, 1, 1, 0), 2.5), 0.001);

        // the y axis is in the gap of the x axis, so use tmirror
        // x: (0,1) -> (0.25,1) tswitch=0.224 tlimit=0.292 tmirror=1.707
        // y: (0,0) -> (1.00,0) tswitch=1.264 tlimit=nan tmirror=nan
        // note u = 2
        assertEquals(1.707, RRTStar7.tOptimal(s(0, 1, 0, 0), s(0.25, 1, 1, 0), 2.0), 0.001);

        // another gap example
        // x: (0,1) -> (0.25,1) tswitch=0.224 tlimit=0.292 tmirror=1.707
        // y: (0,1) -> (0.50,1) tswitch=0.414 tlimit=1.000 tmirror=1.000
        // note u = 2
        assertEquals(1.707, RRTStar7.tOptimal(s(0, 1, 0, 1), s(0.25, 1, 0.5, 1), 2.0), 0.001);

        // both are above the gap so it just picks the slower tswitch
        // x: (0,2) -> (1.0,2) tswitch=0.449 tlimit=0.585 tmirror=3.414
        // y: (0,1) -> (0.5,1) tswitch=0.414 tlimit=1.000 tmirror=1.000
        assertEquals(0.449, RRTStar7.tOptimal(s(0, 2, 0, 1), s(1, 2, 0.5, 1), 2.0), 0.001);
    }

    @Test
    void testSteer() {
        // x: (0,0) -> (1,0)
        // y: (0,0) -> (1,0)
        Matrix<N4, N1> x_i = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 0, 0, 0, 0 });
        Matrix<N4, N1> x_g = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1, 0, 1, 0 });
        Trajectory phi = RRTStar7.BangBangSteer(x_i, x_g, true);
        assertEquals(100, phi.x.s1.u, 0.001);
        assertEquals(100, phi.x.s1.t, 0.001);
        assertEquals(100, phi.x.s2.u, 0.001);
        assertEquals(100, phi.x.s2.t, 0.001);
        assertEquals(100, phi.y.s1.u, 0.001);
        assertEquals(100, phi.y.s1.t, 0.001);
        assertEquals(100, phi.y.s2.u, 0.001);
        assertEquals(100, phi.y.s2.t, 0.001);

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

        // (0,0) -> (1,0)
        assertEquals(Double.NaN, RRTStar7.tLimit(0, 0, 1, 0, 2.5), 0.001);
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

        // (0,0) -> (1,0)
        assertEquals(Double.NaN, RRTStar7.tMirror(0, 0, 1, 0, 2.5), 0.001);
    }

    @Test
    void quadraticTest() {
        assertEquals(List.of(0.0), RRTStar7.quadratic(1, 0, 0));
        assertEquals(List.of(1.0, -1.0), RRTStar7.quadratic(1, 0, -1));
        // https://en.wikipedia.org/wiki/Quadratic_formula
        assertEquals(List.of(4.0, 1.0), RRTStar7.quadratic(0.5, -2.5, 2));
        assertEquals(List.of(-0.0), RRTStar7.quadratic(0, 1, 0));

    }
}
