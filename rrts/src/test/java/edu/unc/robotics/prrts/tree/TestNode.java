package edu.unc.robotics.prrts.tree;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.graph.Graph;
import org.team100.lib.graph.Link;
import org.team100.lib.graph.Node;
import org.team100.lib.planner.RobotModel;

public class TestNode {

    RobotModel myRobot = new RobotModel() {

        @Override
        public double[] initial() {
            return null;
        }

        @Override
        public boolean goal(double[] config) {
            return false;
        }

        @Override
        public boolean clear(double[] config) {
            return true;
        }

        @Override
        public boolean link(double[] a, double[] b) {
            return true;
        }

    };

    @Test
    void testfoo() {
        Node root = new Node(new double[] { 0, 0 });
        assertArrayEquals(new double[] { 0, 0 }, root.getState());
        assertNull(root.getIncoming());

        Node node1 = new Node(new double[] { 1, 0 });
        Link link1 = Graph.newLink(root, node1, 1);
        assertArrayEquals(new double[] { 1, 0 }, node1.getState());
        assertEquals(root, node1.getIncoming().get_source());
        assertEquals(1, link1.get_linkDist(), 0.1);
        assertEquals(1, link1.get_pathDist(), 0.1);

        Node node2 = new Node(new double[] { 2, 0 });
        Link link2 = Graph.newLink(node1, node2, 1);
        assertArrayEquals(new double[] { 2, 0 }, node2.getState());
        assertEquals(node1, node2.getIncoming().get_source());
        assertEquals(1, link2.get_linkDist(), 0.1);
        assertEquals(2, link2.get_pathDist(), 0.1);

        // now add a second child to the root. what happens?
        Node node3 = new Node(new double[] { 0, 1 });
        Link link3 = Graph.newLink(root, node3, 1);
        assertArrayEquals(new double[] { 0, 1 }, node3.getState());
        assertEquals(root, node3.getIncoming().get_source());
        assertEquals(1, link3.get_linkDist(), 0.1);
        assertEquals(1, link3.get_pathDist(), 0.1);

        // where does the third child go?
        Node node4 = new Node(new double[] { 0, -1 });
        Link link4 = Graph.newLink(root, node4, 1);
        assertArrayEquals(new double[] { 0, -1 }, node4.getState());
        assertEquals(root, node4.getIncoming().get_source());
        assertEquals(1, link4.get_linkDist(), 0.1);
        assertEquals(1, link4.get_pathDist(), 0.1);

        Node node5 = new Node(new double[] {1,1});
        Link link5 = Graph.newLink(node1, node5, 1);
        assertArrayEquals(new double[] { 1, 1 }, node5.getState());
        assertEquals(node1, node5.getIncoming().get_source());
        assertEquals(1, link5.get_linkDist(), 0.1);
        assertEquals(2, link5.get_pathDist(), 0.1);
    }


}
