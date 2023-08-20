package edu.unc.robotics.prrts.tree;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

public class TestNode {
    @Test
    void testfoo() {
        Node root = new Node(new double[] { 0, 0 }, false);
        assertArrayEquals(new double[] { 0, 0 }, root.get_config());
        assertNull(root.get_incoming().get_source());

        Node node1 = new Node(new double[] { 1, 0 }, false, 1.0, root);
        assertArrayEquals(new double[] { 1, 0 }, node1.get_config());
        assertEquals(root, node1.get_incoming().get_source());

        Node node2 = new Node(new double[] { 2, 0 }, false, 1.0, node1);
        assertArrayEquals(new double[] { 2, 0 }, node2.get_config());
        assertEquals(node1, node2.get_incoming().get_source());

        // now add a second child to the root. what happens?
        Node node3 = new Node(new double[] { 0, 1 }, false, 1.0, root);
        assertArrayEquals(new double[] { 0, 1 }, node3.get_config());
        assertEquals(root, node3.get_incoming().get_source());

        // where does the third child go?
        Node node4 = new Node(new double[] { 0, -1 }, false, 1.0, root);
        assertArrayEquals(new double[] { 0, -1 }, node4.get_config());
        assertEquals(root, node4.get_incoming().get_source());


    }

}
