package org.team100.lib.prrts;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Set;

import org.jgrapht.Graph;
import org.jgrapht.graph.SimpleDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.junit.jupiter.api.Test;

/**
 * Try out some alternatives for graph library basics
 */
public class GraphLibTest {
    @Test
    public void testJGraphT() {
        Graph<String, DefaultEdge> g = new SimpleDirectedGraph<>(DefaultEdge.class);
        g.addVertex("root");
        g.addVertex("one");
        g.addVertex("two");
        g.addEdge("root", "one");
        DefaultEdge e2 = g.addEdge("root", "two");
        Set<DefaultEdge> parent_edges = g.incomingEdgesOf("two");
        assertArrayEquals(new DefaultEdge[] {e2}, parent_edges.toArray(new DefaultEdge[0]));
        assertEquals("root", g.getEdgeSource(e2));
        assertEquals("root", g.getEdgeSource(g.incomingEdgesOf("one").iterator().next()));

        // move parent
        g.removeEdge(e2);
        g.addEdge("one","two");
        assertEquals("one", g.getEdgeSource(g.incomingEdgesOf("two").iterator().next()));
    }

}
