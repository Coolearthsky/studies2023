package edu.unc.robotics.prrts;

import java.util.Arrays;

public class ArrayState implements State {
    private final double[] config; // TODO change the name

    public ArrayState(double[] config) {
        this.config = config;
    }

    public double[] getConfig() {
        return config;
    }

    public double get(int axis) {
        return config[axis];
    }

    public void set(int axis, double value) {
        config[axis] = value;
    }

    public ArrayState copy() {
        double[] out = new double[config.length];
        System.arraycopy(config, 0, out, 0, config.length);
        return new ArrayState(out);
    }

    @Override
    public boolean equals(State other) {
        if (!(other instanceof ArrayState))
            return false;
        ArrayState arrayState = (ArrayState) other;
        return Arrays.equals(config, arrayState.config);
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(config);
    }

}
