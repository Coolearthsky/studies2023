package edu.unc.robotics.prrts.tree;

import java.util.concurrent.atomic.AtomicReference;

import edu.unc.robotics.prrts.State;

/** Just shrinking the enormous PRRTStar class */
public class Link<T extends State> {
    public final Node<T> node;
    public final double linkDist;
    public final double pathDist;

    public final AtomicReference<Link<T>> parent = new AtomicReference<>();
    private final AtomicReference<Link<T>> firstChild = new AtomicReference<>();
    final AtomicReference<Link<T>> nextSibling = new AtomicReference<>();

    public Link(Node<T> root) {
        this.node = root;
        this.linkDist = 0;
        this.pathDist = 0;
    }

    public Link(Node<T> node, double linkDist, Link<T> parent) {
        this.node = node;
        this.linkDist = linkDist;
        this.pathDist = parent.pathDist + linkDist;
        this.parent.set(parent);
    }

    boolean setParent(Link<T> oldValue, Link<T> newValue) {
        return parent.compareAndSet(oldValue, newValue);
    }

    boolean setFirstChild(Link<T> oldValue, Link<T> newValue) {
        return firstChild.compareAndSet(oldValue, newValue);
    }

    boolean setNextSibling(Link<T> oldValue, Link<T> newValue) {
        return nextSibling.compareAndSet(oldValue, newValue);
    }

    void addChild(Link<T> child) {
        Link<T> expected = null;
        Link<T> nextSibling;

        do {
            nextSibling = firstChild.get();

            if (!child.setNextSibling(expected, nextSibling)) {
                assert false : "nextSibling initialized to unexpected value";
            }

            expected = nextSibling;
        } while (!setFirstChild(nextSibling, child));
    }

    public boolean isExpired() {
        return node.link.get() != this;
    }

    public Link<T> removeFirstChild() {
        Link<T> child;
        Link<T> sibling;

        do {
            child = firstChild.get();
            if (child == null) {
                return null;
            }

            sibling = child.nextSibling.get();
        } while (!setFirstChild(child, sibling));

        if (!child.setNextSibling(sibling, null)) {
            assert false : "sibling changed after removal";
        }

        return child;
    }

    public boolean removeChild(final Link<T> child) {
        Link<T> sibling;
        Link<T> n;
        Link<T> p;

        assert child.isExpired() : "removing unexpired child";
        assert child.parent.get() == this : "not child's parent";

        outer: for (;;) {
            n = firstChild.get();

            if (n == child) {
                sibling = child.nextSibling.get();

                if (setFirstChild(child, sibling)) {
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
                    n = n.nextSibling.get();

                if (n == null) {
                    return false;
                }

                if (n == child) {
                    sibling = child.nextSibling.get();

                    // TODO: double check this logic. could the child
                    // now be the first element in the list?

                    if (p.setNextSibling(child, sibling)) {
                        break outer;
                    } else {
                        break;
                    }
                }
            }
        }

        child.setNextSibling(sibling, null);

        return true;
    }

}
