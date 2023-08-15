package edu.unc.robotics.prrts.kdtree;

import java.util.ArrayList;
import java.util.List;

/**
 * KDTree
 *
 * @author jeffi
 */
public class KDTree<V> {
    private final KDModel _model;
    private final KDNode<V> _root;

    public KDTree(KDModel model, double[] rootConfig, V rootValue) {
        _model = model;
        _root = new KDNode<V>(rootConfig, rootValue);
    }

    public Iterable<V> values() {
        List<V> list = new ArrayList<V>();
        buildList(list, _root);
        return list;
    }

    private void buildList(List<V> list, KDNode<V> node) {
        if (node == null)
            return;

        list.add(node.value);
        buildList(list, node.a.get());
        buildList(list, node.b.get());
    }

    public KDTraversal<V> newTraversal() {
        return new Traversal<V>(_model, _root, _model.dimensions());
    }
}
