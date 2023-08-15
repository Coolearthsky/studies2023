package edu.unc.robotics.prrts.tree;

import java.util.concurrent.atomic.AtomicReference;

/** Just shrinking the enormous PRRTStar class */
public class Link {
    // tail of this edge
    private final Node _node;
    // length of this edge
    private final double _linkDist;
    // total path length so far
    private final double _pathDist;
    private final Link _parent;
    private final AtomicReference<Link> _firstChild;
    private final AtomicReference<Link> _nextSibling;

    public Link(Node root) {
        this(root, 0, 0, null, null, null);
    }

    public Link(Node node, double linkDist, Link parent) {
        this(node, linkDist, parent._pathDist + linkDist, parent, null, null);
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

    private Link(Node root,
            double linkDist,
            double pathDist,
            Link parent,
            Link firstChild,
            Link nextSibling) {
        _node = root;
        _linkDist = linkDist;
        _pathDist = pathDist;
        _parent = parent;
        _firstChild = new AtomicReference<>(firstChild);
        _nextSibling = new AtomicReference<>(nextSibling);
    }

}
