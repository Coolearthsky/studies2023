package team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;

/**
 * Illustrates delayed measurements and what to do with them.
 */
public class EstimatorLatencyTest {
    static final double kDelta = 0.001;
    // static final double kDt = 0.02;

    @Test
    public void testConstantVelocity() {
        System.out.println("\n\nCONSTANT VELOCITY");
        final double actualTimeQuantum = 0.002;
        final int stepsPerFilterQuantum = 10;
        final double filterQuantum = actualTimeQuantum * stepsPerFilterQuantum;

        // no control input, constant angular velocity
        final AngleEstimator observer = new AngleEstimator(
                VecBuilder.fill(0.1, 0.1),
                VecBuilder.fill(0.1, 0.1),
                filterQuantum);

        // initially in motion
        observer.reset();
        observer.setXhat(VecBuilder.fill(0, 1));
        double predictedAngle = observer.getXhat(0);
        double predictedVelocity = observer.getXhat(1);
        assertEquals(0, predictedAngle, kDelta);
        assertEquals(1, predictedVelocity, kDelta);

        // constant zero input
        final double u = 0;

        System.out.printf("%8s %8s %8s %8s %8s %8s %8s %8s %8s %8s\n",
                "step", "aTime", "aPos", "aVel", "oPos", "oVel", "pPos", "pVel", "ePos", "eVel");
        for (int step = 0; step < 500; ++step) {
            double actualTime = step * actualTimeQuantum;
            final double actualVelocity = 1;
            double actualAngle = actualVelocity * actualTime;
            // perfect sensors for now
            double observedAngle = actualAngle;
            double observedVelocity = actualVelocity;
            if (step % stepsPerFilterQuantum == 0) {
                // time to talk to the filter
                observer.correctAngle(u, observedAngle);
                observer.correctVelocity(u, observedVelocity);
                // the prediction is for the state at the end of the next quantum.
                observer.predictState(u, filterQuantum);
            }
            predictedAngle = observer.getXhat(0);
            predictedVelocity = observer.getXhat(1);
            // the errors should be zero at the *next* filter quantum.
            // note negative error means the data is below the prediction i.e. prediction
            // should be more negative
            double errAngle = actualAngle - predictedAngle;
            double errVelocity = actualVelocity - predictedVelocity;
            System.out.printf("%8d %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n",
                    step, actualTime, actualAngle, actualVelocity, observedAngle, observedVelocity, predictedAngle,
                    predictedVelocity, errAngle, errVelocity);
        }
    }

    @Test
    public void testConstantAcceleration() {
        System.out.println("\n\nCONSTANT ACCELERATION");
        final double actualTimeQuantum = 0.002;
        final int stepsPerFilterQuantum = 10;
        final double filterQuantum = actualTimeQuantum * stepsPerFilterQuantum;

        final AngleEstimator observer = new AngleEstimator(
                VecBuilder.fill(0.1, 0.1),
                VecBuilder.fill(0.1, 0.1),
                filterQuantum);

        // constant torque.
        final double u = 1;
        // zero initial velocity.
        double actualVelocity = 0;
        // zero initial angle.
        double actualAngle = 0;

        observer.reset();
        observer.setXhat(VecBuilder.fill(actualAngle, actualVelocity));
        assertEquals(0, observer.getXhat(0), kDelta);
        assertEquals(0, observer.getXhat(1), kDelta);

        System.out.printf("%8s %8s %8s %8s %8s %8s %8s %8s %8s %8s\n",
                "step", "aTime", "aPos", "aVel", "oPos", "oVel", "pPos", "pVel", "ePos", "eVel");

        // initial conditions
        double actualTime = 0;
        double predictedAngle = observer.getXhat(0);
        double predictedVelocity = observer.getXhat(1);

        for (int step = 0; step < 500; ++step) {
            actualTime = step * actualTimeQuantum;
            actualVelocity = u * actualTime;
            actualAngle = 0.5 * u * Math.pow(actualTime, 2);
            // perfect sensors for now
            double observedAngle = actualAngle;
            double observedVelocity = actualVelocity;
            if (step % stepsPerFilterQuantum == 0) {
                // time to talk to the filter
                observer.correctAngle(u, observedAngle);
                observer.correctVelocity(u, observedVelocity);
                // the prediction is for the state at the end of the next quantum.
                observer.predictState(u, filterQuantum);
            }
            predictedAngle = observer.getXhat(0);
            predictedVelocity = observer.getXhat(1);
            // the errors should be zero at the *next* filter quantum.
            // note negative error means the data is below the prediction i.e. prediction
            // should be more negative
            double errAngle = actualAngle - predictedAngle;
            double errVelocity = actualVelocity - predictedVelocity;
            System.out.printf("%8d %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n",
                    step, actualTime, actualAngle, actualVelocity, observedAngle, observedVelocity, predictedAngle,
                    predictedVelocity, errAngle, errVelocity);
        }
    }

    @Test
    public void testSinusoidal() {
        System.out.println("\n\nSINUSOIDAL");
        final double timeScale = 1;
        final double mass = 1;
        final double actualTimeQuantum = 0.002;
        final int stepsPerFilterQuantum = 10;
        final double filterQuantum = actualTimeQuantum * stepsPerFilterQuantum;

        final AngleEstimator observer = new AngleEstimator(
                VecBuilder.fill(0.1, 0.1),
                VecBuilder.fill(0.1, 0.1),
                filterQuantum);

        double actualVelocity = 0;
        double actualAngle = 1;

        observer.reset();
        observer.setXhat(VecBuilder.fill(actualAngle, actualVelocity));
        assertEquals(1, observer.getXhat(0), kDelta);
        assertEquals(0, observer.getXhat(1), kDelta);

        System.out.printf("%8s %8s %8s %8s %8s %8s %8s %8s %8s %8s %8s %8s\n",
                "step", "aTime", "aPos", "aVel", "oPos", "oVel", "oAcc", "pPos", "pVel", "ePos", "eVel", "cont");

        // initial conditions
        double actualTime = 0;
        double predictedAngle = observer.getXhat(0);
        double predictedVelocity = observer.getXhat(1);
        // control effort
        double u = 0;
        double observedAngle = 0;
        double observedVelocity = 0;
        double observedAcceleration = 0;
        for (int step = 0; step < 2000; ++step) {
            actualTime = step * actualTimeQuantum;
            double actualAcceleration = -1.0 * Math.cos(timeScale * actualTime);
            // actualVelocity = u * actualTime;
            actualVelocity = -1.0 * Math.sin(timeScale * actualTime);
            // actualAngle = 0.5 * u * Math.pow(actualTime, 2);
            actualAngle = Math.cos(timeScale * actualTime);
            observedAcceleration = (actualVelocity - observedVelocity) / actualTimeQuantum;
            // perfect sensors
            observedVelocity = actualVelocity;
            observedAngle = actualAngle;
            if (step % stepsPerFilterQuantum == 0) {
                u = mass * actualAcceleration;
                observer.correctAngle(u, observedAngle);
                observer.correctVelocity(u, observedVelocity);
                // the prediction is for the state at the end of the next quantum.
                observer.predictState(u, filterQuantum);
            }
            predictedAngle = observer.getXhat(0);
            predictedVelocity = observer.getXhat(1);
            // the errors should be zero at the *next* filter quantum.
            // note negative error means the data is below the prediction i.e. prediction
            // should be more negative
            double errAngle = actualAngle - predictedAngle;
            double errVelocity = actualVelocity - predictedVelocity;
            System.out.printf("%8d %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n",
                    step, actualTime, actualAngle, actualVelocity,
                    observedAngle, observedVelocity, observedAcceleration,
                    predictedAngle, predictedVelocity, errAngle, errVelocity, u);
        }
    }
}
