package org.team100.lib.simulation;

public class CompleteState {
    private static final double kSecPerUsec = 1e-6;
    // int step;
    long systemTimeMicrosec; // from FPGAtime

    double actualTimeSec() {
        return systemTimeMicrosec * kSecPerUsec;
    }

    double actualPosition;
    double actualVelocity;
    double actualAcceleration;

    double observationTimeSec; // valid time this observation represents
    double observedPosition;
    double observedVelocity;
    double observedAcceleration; // for these tests this is intended to match u.

    double predictionTimeSec; // time in the future this prediction is intended for
    double predictedPosition;
    double predictedVelocity;

    double residualPosition;
    double residualVelocity;

    double controlU;

    public void init(
            double initialPosition,
            double initialVelocity,
            double initialAcceleration) {
        systemTimeMicrosec = 0l;

        actualPosition = initialPosition;
        actualVelocity = initialVelocity;
        actualAcceleration = initialAcceleration;

        observedPosition = initialPosition;
        observedVelocity = initialVelocity;
        observedAcceleration = initialAcceleration;

        predictedPosition = initialPosition;
        predictedVelocity = initialVelocity;

        residualPosition = 0;
        residualVelocity = 0;

        controlU = initialAcceleration;
    }


    public String header() {
        return "     sysTime,   actualTime, " +
                "   actualPos,    actualVel,    actualAcc, " +
                " observedPos,  observedVel,  observedAcc, " +
                "predictedPos, predictedVel, " +
                " residualPos,  residualVel, " +
                "    controlU";
    }

    public String toString() {
        String format = "%12d, %12.3f, " +
                "%12.3f, %12.3f, %12.3f, " +
                "%12.3f, %12.3f, %12.3f, " +
                "%12.3f, %12.3f, " +
                "%12.3f, %12.3f, " +
                "%12.3f";
        return String.format(format,
                systemTimeMicrosec,
                actualTimeSec(),
                actualPosition,
                actualVelocity,
                actualAcceleration,
                observedPosition,
                observedVelocity,
                observedAcceleration,
                predictedPosition,
                predictedVelocity,
                residualPosition,
                residualVelocity,
                controlU);

    }
}
