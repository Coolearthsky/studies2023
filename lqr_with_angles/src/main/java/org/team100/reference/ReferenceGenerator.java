package org.team100.reference;

import org.team100.estimator.ExtendedAngleEstimator;

/**
 * ReferenceGenerator provides feasible trajectories of full state references
 * for control, based on full, partial, and/or infeasible state input, and a
 * supplied estimator.
 * 
 * For path planning, supply the endpoint and constraints. A timed trajectory
 * will be created that satisfies the input; it can be repeatedly sampled.
 * Updating the goal or constraints will regenerate the full trajectory.
 * 
 * Manual full-state control works the sme way, e.g. for rotation snaps.
 * 
 * Manual partial-state control, e.g. supplying velocity, uses the estimator to
 * supplement the input.
 * 
 * This is inspired by 254's SetpointGenerator and SwerveSetpointGenerator.
 */
public class ReferenceGenerator {
    public static class Constraint {
        public double jerk;
        public double accel;
        public double vel;
    }
    public static class State {
        public double position;
        public double velocity;
    }
    public static class Profile {
        
    }
    private final ExtendedAngleEstimator estimator;

    public ReferenceGenerator(ExtendedAngleEstimator estimator) {
        this.estimator = estimator;
    }

    

}
