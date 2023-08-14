package edu.unc.robotics.prrts;

public interface State {
    boolean equals(State other);

    /**
     * This is just the KD Tree to do its dividing-the-space thing.
     * 
     * Maybe there's a better way to do this, so that other classes can't see it.
     * 
     */
    double get(int axis);
    void set(int axis, double value);
}
