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
        assertNull(root.get_link().get().get_parent());
        assertNull(root.get_link().get()._firstChild.get());
        assertNull(root.get_link().get()._nextSibling.get());

        Node node1 = new Node(new double[] { 1, 0 }, false, 1.0, root.get_link().get());
        assertArrayEquals(new double[] { 1, 0 }, node1.get_config());
        // we explicitly set the parent, so there it is.
        assertEquals(root.get_link().get(), node1.get_link().get().get_parent());
        // the parent should now have this node as the first child
        assertEquals(node1, root.get_link().get()._firstChild.get().get_node());
        assertNull(node1.get_link().get()._firstChild.get());
        assertNull(node1.get_link().get()._nextSibling.get());

        Node node2 = new Node(new double[] { 2, 0 }, false, 1.0, node1.get_link().get());
        assertArrayEquals(new double[] { 2, 0 }, node2.get_config());
        assertEquals(node1.get_link().get(), node2.get_link().get().get_parent());
        assertEquals(node2, node1.get_link().get()._firstChild.get().get_node());
        assertNull(node2.get_link().get()._firstChild.get());
        assertNull(node2.get_link().get()._nextSibling.get());

        // now add a second child to the root. what happens?
        Node node3 = new Node(new double[] {0, 1}, false, 1.0, root.get_link().get());
        assertArrayEquals(new double[] { 0, 1 }, node3.get_config());
        // parent reference was set explicitly
        assertEquals(root.get_link().get(), node3.get_link().get().get_parent());
        // we bump the first child
        assertEquals(node3, root.get_link().get()._firstChild.get().get_node());
        // the previous first child is now our sibling
        assertEquals(node1, node3.get_link().get()._nextSibling.get().get_node());

        // where does the third child go?
        Node node4 = new Node(new double[] {0, -1}, false, 1.0, root.get_link().get());
        assertArrayEquals(new double[] { 0, -1 }, node4.get_config());
        // parent reference was set explicitly
        assertEquals(root.get_link().get(), node4.get_link().get().get_parent());
        // bump the first child again
        assertEquals(node4, root.get_link().get()._firstChild.get().get_node());
        // previous first child is now my sibling
        assertEquals(node3, node4.get_link().get()._nextSibling.get().get_node());
        // one before that is another sibling
        assertEquals(node1, node3.get_link().get()._nextSibling.get().get_node());

    }

}
