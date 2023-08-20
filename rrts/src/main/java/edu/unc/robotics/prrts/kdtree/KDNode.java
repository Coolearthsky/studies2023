package edu.unc.robotics.prrts.kdtree;

public final class KDNode<V> {
    private final double[] config;
    private final V value;
    private KDNode<V> a;
    private KDNode<V> b;

    public KDNode(double[] c, V v) {
        if (c == null)
            throw new IllegalArgumentException("null config");
        if (v == null)
            throw new IllegalArgumentException("null value");
        config = c;
        value = v;
    }

    void setA(KDNode<V> n) {
        a = n;
    }

    void setB(KDNode<V> n) {
        b = n;
    }

    public KDNode<V> getA() {
        return a;
    }

    public KDNode<V> getB() {
        return b;
    }

    public double[] getConfig() {
        return config;
    }

    public V getValue() {
        return value;
    }
}