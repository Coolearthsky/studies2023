package org.team100.lib.simulation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public abstract class Scenario {

    abstract String label();

    abstract double position(double timeSec);

    abstract double velocity(double timeSec);

    abstract double acceleration(double timeSec);

    /**
     * Specify the desired future state, could be from a trajectory
     */
    Vector<N2> getR(double futureTimeSec) {
        double referencePosition = position(futureTimeSec);
        double referenceVelocity = velocity(futureTimeSec);
        return VecBuilder.fill(referencePosition, referenceVelocity);
    }

    /**
     * analytical future state derivative, could be from a trajectory
     */
    Vector<N2> getRDot(double futureTimeSec) {
        double referenceVelocity = velocity(futureTimeSec);
        double referenceAcceleration = acceleration(futureTimeSec);
        return VecBuilder.fill(referenceVelocity, referenceAcceleration);
    }
}