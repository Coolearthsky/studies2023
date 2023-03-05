package frc.robot;

import java.util.TreeMap;

// i just recorded some x and y and found the max hypot for each angle bucket.
// TODO: narrower buckets.
// see https://docs.google.com/spreadsheets/d/1pxCVCgM7VJcN_mcnjRJJQ4sHIY2OQMwu4ETg4na1QX4/edit#gid=1691413595

public class MaxHypot {
    public static final TreeMap<Double, Double> tree = new TreeMap<Double, Double>();

    static {
        tree.put(-Double.MAX_VALUE, 1.007);
        tree.put(-3.0, 1.007);
        tree.put(-2.8, 1.059);
        tree.put(-2.6, 1.107);
        tree.put(-2.4, 1.098);
        tree.put(-2.2, 1.096);
        tree.put(-2.0, 1.158);
        tree.put(-1.8, 1.094);
        tree.put(-1.6, 1.026);
        tree.put(-1.4, 1.009);
        tree.put(-1.2, 1.067);
        tree.put(-1.0, 1.122);
        tree.put(-0.8, 1.108);
        tree.put(-0.6, 1.114);
        tree.put(-0.4, 1.154);
        tree.put(-0.2, 1.084);
        tree.put(0.0, 1.02);
        tree.put(0.2, 1.018);
        tree.put(0.4, 1.076);
        tree.put(0.6, 1.084);
        tree.put(0.8, 1.082);
        tree.put(1.0, 1.081);
        tree.put(1.2, 1.124);
        tree.put(1.4, 1.073);
        tree.put(1.6, 1.01);
        tree.put(1.8, 1.025);
        tree.put(2.0, 1.094);
        tree.put(2.2, 1.126);
        tree.put(2.4, 1.112);
        tree.put(2.6, 1.122);
        tree.put(2.8, 1.166);
        tree.put(3.0, 1.059);
        tree.put(3.2, 1.009);
        tree.put(Double.MAX_VALUE, 1.009);
    }
}
