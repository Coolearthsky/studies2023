package edu.unc.robotics.prrts.tree;

import java.util.concurrent.atomic.AtomicReference;

/**
 * A doubly-connected directed graph which also has a linked list of siblings.
 */
public class Link {
    /** head of this link */
    private final Node _node;
    /** length, i.e. cost, of this edge */
    private final double _linkDist;
    /** total path length, i.e. cost, so far */
    private final double _pathDist;
    /**
     * tail of this link
     * it's weird that this is another link instead of a node. maybe that's some
     * sort of infinitesimal speedup?
     */
    private final Link _parent;
    /** link to a node that this node is the parent of, if any */
    final AtomicReference<Link> _firstChild;
    /** link to a node with the same parent as this node */
    final AtomicReference<Link> _nextSibling;

    public Link(Node root) {
        this(root, 0, 0, null);
    }

    /**
     * Create a new link pointing at the node, linkDist away from parent.
     * 
     * @param node     head of this link
     * @param linkDist distance to the parent
     * @param parent   link whose head is the parent node
     */
    public Link(Node node, double linkDist, Link parent) {
        this(node, linkDist, parent._pathDist + linkDist, parent);
    }

    public void addChild(Link child) {
        Link expected = null;
        Link nextSibling;

        do {
            nextSibling = _firstChild.get();

            if (!child._nextSibling.compareAndSet(expected, nextSibling)) {
                assert false : "nextSibling initialized to unexpected value";
            }

            expected = nextSibling;
        } while (!_firstChild.compareAndSet(nextSibling, child));
    }

    public boolean isExpired() {
        return _node.get_link().get() != this;
    }

    public Link removeFirstChild() {
        Link child;
        Link sibling;

        do {
            child = _firstChild.get();
            if (child == null) {
                return null;
            }

            sibling = child._nextSibling.get();
        } while (!_firstChild.compareAndSet(child, sibling));

        if (!child._nextSibling.compareAndSet(sibling, null)) {
            assert false : "sibling changed after removal";
        }

        return child;
    }

    public boolean removeChild(final Link child) {
        Link sibling;
        Link n;
        Link p;

        assert child.isExpired() : "removing unexpired child";
        assert child._parent == this : "not child's parent";

        outer: for (;;) {
            n = _firstChild.get();

            if (n == child) {
                sibling = child._nextSibling.get();

                if (_firstChild.compareAndSet(child, sibling)) {
                    break;
                } else {
                    continue;
                }
            }

            if (n == null) {
                return false;
            }

            for (;;) {
                p = n;

                if (n != null)
                    n = n._nextSibling.get();

                if (n == null) {
                    return false;
                }

                if (n == child) {
                    sibling = child._nextSibling.get();

                    // TODO: double check this logic. could the child
                    // now be the first element in the list?

                    if (p._nextSibling.compareAndSet(child, sibling)) {
                        break outer;
                    } else {
                        break;
                    }
                }
            }
        }

        child._nextSibling.compareAndSet(sibling, null);

        return true;
    }

    public Node get_node() {
        return _node;
    }

    public double get_linkDist() {
        return _linkDist;
    }

    public double get_pathDist() {
        return _pathDist;
    }

    public Link get_parent() {
        return _parent;
    }

    //////////////////////////////////////////////////////////////////

    /** @param node tail of this link */
    private Link(Node node,
            double linkDist,
            double pathDist,
            Link parent) {
        _node = node;
        _linkDist = linkDist;
        _pathDist = pathDist;
        _parent = parent;
        _firstChild = new AtomicReference<>(null);
        _nextSibling = new AtomicReference<>(null);
    }

}
