package team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

/**
 * Illustrates delayed measurements and what to do with them.
 * 
 * TODO: actually delay the measurements.  :-)
 * 
 * see https://docs.google.com/spreadsheets/d/1miehTmvbdRFs49wy2x8u_3SqVKrNw-oJB5Dv5-uQYo0
 */
public class EstimatorLatencyTest {
    private static final double kDelta = 0.001;
    private static final double actualTimeQuantum = 0.002;
    private static final int stepsPerFilterQuantum = 10;
    private static final double filterTimeQuantum = actualTimeQuantum * stepsPerFilterQuantum;

    public static class CompleteState {
        int step;
        double actualTime;

        double actualPosition;
        double actualVelocity;
        double actualAcceleration;

        double observedPosition;
        double observedVelocity;
        double observedAcceleration;

        double predictedPosition;
        double predictedVelocity;

        double residualPosition;
        double residualVelocity;

        double controlU;
    }

    private static AngleEstimator newObserver(double initialPosition, double initialVelocity) {
        final AngleEstimator observer = new AngleEstimator(
                VecBuilder.fill(0.1, 0.1),
                // VecBuilder.fill(0.1, 0.1),
                VecBuilder.fill(0.01, 0.01),
                filterTimeQuantum);
        observer.reset();
        observer.setXhat(VecBuilder.fill(initialPosition, initialVelocity));
        assertEquals(initialPosition, observer.getXhat(0), kDelta);
        assertEquals(initialVelocity, observer.getXhat(1), kDelta);
        return observer;
    }

    private static LinearSystem<N2, N1, N2> newPlant() {
        final Matrix<N2, N2> A = Matrix.mat(Nat.N2(), Nat.N2()).fill(0, 1, 0, 0);
        final Matrix<N2, N1> B = Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 1);
        final Matrix<N2, N2> C = Matrix.mat(Nat.N2(), Nat.N2()).fill(1, 0, 0, 1);
        final Matrix<N2, N1> D = Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 0);
        final LinearSystem<N2, N1, N2> plant = new LinearSystem<>(A, B, C, D);
        return plant;
    }

    private static AngleController newController(LinearSystem<N2, N1, N2> plant) {
        // final Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        final Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.01);
        final Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        final AngleController controller = new AngleController(
                plant,
                stateTolerance,
                controlTolerance,
                filterTimeQuantum);
        controller.reset();
        // assertEquals(572, controller.getK().get(0, 0), 1.0);
        assertEquals(49, controller.getK().get(0, 0), 1.0);
        // assertEquals(44, controller.getK().get(0, 1), 1.0);
        assertEquals(50, controller.getK().get(0, 1), 1.0);
        return controller;
    }

    /** Constant velocity means no torque so no controller output. */
    @Test
    public void testConstantVelocity() {
        System.out.println("\n\nCONSTANT VELOCITY");

        printHeader();

        CompleteState state = new CompleteState();

        state.actualTime = 0;

        state.actualPosition = 0;
        state.actualVelocity = 1;
        state.actualAcceleration = 0;

        state.observedPosition = 0;
        state.observedVelocity = 1;
        state.observedAcceleration = 0;

        state.predictedPosition = 0;
        state.predictedVelocity = 1;

        state.residualPosition = 0;
        state.residualVelocity = 0;

        state.controlU = 0;

        final AngleEstimator observer = newObserver(0, 1);
        final LinearSystem<N2, N1, N2> plant = newPlant();
        final LinearPlantInversionFeedforward<N2, N1, N2> feedforward = new LinearPlantInversionFeedforward<>(plant,
                filterTimeQuantum);
        // initial velocity
        feedforward.calculate(VecBuilder.fill(0, 1));
        final AngleController controller = newController(plant);

        for (state.step = 0; state.step < 2000; ++state.step) {
            state.actualTime = state.step * actualTimeQuantum;

            state.actualAcceleration = 0;
            state.actualVelocity = 1;
            state.actualPosition = MathUtil.angleModulus(state.actualVelocity * state.actualTime);

            state.observedPosition = state.actualPosition;
            state.observedVelocity = state.actualVelocity;
            state.observedAcceleration = state.actualAcceleration;

            if (state.step % stepsPerFilterQuantum == 0) {
                // talk to the observer and controller periodically
                // the goal is to decide what to do for the next quantum,
                // so here we are looking at the future.

                // 1. correct the observer with current measurements.
                // right now these measurements represent the current instant
                // TODO: add measurement delay
                observer.correctAngle(state.controlU, state.observedPosition);
                observer.correctVelocity(state.controlU, state.observedVelocity);

                // 2. predict the expected future state.
                observer.predictState(state.controlU, filterTimeQuantum);

                // this is the predicted future state given the previous control
                Matrix<N2, N1> nextXhat = observer.getXhat();

                // 3. specify the desired future state.
                // (here we expect constant velocity but it could be, say, a trajectory or
                // trapezoid.)
                double setpointPosition = MathUtil
                        .angleModulus(state.actualVelocity * (state.actualTime + filterTimeQuantum));
                double setpointVelocity = 1;
                Vector<N2> setpoint = VecBuilder.fill(setpointPosition, setpointVelocity);

                // 4. calculate control output.
                // compare the desired and expected states, and produce output to nudge the
                // expectation towards the desire
                // TODO: actually drive the actual state using this output
                controller.calculate(nextXhat, setpoint);

                // combine LQR and FF
                Matrix<N1, N1> u = controller.getU();
                Matrix<N1, N1> uff = feedforward.calculate(setpoint);
                state.controlU = u.plus(uff).get(0, 0);
            }
            state.predictedPosition = observer.getXhat(0);
            state.predictedVelocity = observer.getXhat(1);
            // the errors should be zero at the *next* filter quantum.
            // note negative error means the data is below the prediction i.e. prediction
            // should be more negative
            state.residualPosition = MathUtil.angleModulus(state.actualPosition - state.predictedPosition);
            state.residualVelocity = state.actualVelocity - state.predictedVelocity;

            printRow(state);
        }
    }

    @Test
    public void testConstantAcceleration() {
        System.out.println("\n\nCONSTANT ACCELERATION");

        printHeader();

        CompleteState state = new CompleteState();

        state.actualTime = 0;

        state.actualPosition = 0;
        state.actualVelocity = 0;
        state.actualAcceleration = 1;

        state.observedPosition = 0;
        state.observedVelocity = 0;
        state.observedAcceleration = 1;

        state.predictedPosition = 0;
        state.predictedVelocity = 0;

        state.residualPosition = 0;
        state.residualVelocity = 0;

        // initial control
        state.controlU = 1;

        final AngleEstimator observer = newObserver(0, 0);
        final LinearSystem<N2, N1, N2> plant = newPlant();
        final LinearPlantInversionFeedforward<N2, N1, N2> feedforward = new LinearPlantInversionFeedforward<>(plant,
                filterTimeQuantum);
        // initial is zeros
        feedforward.calculate(VecBuilder.fill(0, 0));
        final AngleController controller = newController(plant);

        for (state.step = 0; state.step < 2000; ++state.step) {
            state.actualTime = state.step * actualTimeQuantum;

            state.actualAcceleration = 1;
            state.actualVelocity = state.actualAcceleration * state.actualTime;
            state.actualPosition = MathUtil.angleModulus(Math.pow(state.actualTime, 2) / 2);

            state.observedPosition = state.actualPosition;
            state.observedVelocity = state.actualVelocity;
            state.observedAcceleration = state.actualAcceleration;

            if (state.step % stepsPerFilterQuantum == 0) {
                // talk to the observer and controller periodically
                // the goal is to decide what to do for the next quantum,
                // so here we are looking at the future.

                // 1. correct the observer with current measurements.
                // right now these measurements represent the current instant
                // (TODO: add measurement delay)
                observer.correctAngle(state.controlU, state.observedPosition);
                observer.correctVelocity(state.controlU, state.observedVelocity);

                // 2. predict the expected future state.
                observer.predictState(state.controlU, filterTimeQuantum);

                // this is the predicted future state given the previous control
                Matrix<N2, N1> nextXhat = observer.getXhat();

                // 3. specify the desired future state.
                // (here we magically apply the actual state)
                double setpointPosition = Math.pow(state.actualTime + filterTimeQuantum, 2) / 2;
                double setpointVelocity = state.actualTime + filterTimeQuantum;
                Vector<N2> setpoint = VecBuilder.fill(setpointPosition, setpointVelocity);

                // 4. calculate control output.
                // compare the desired and expected states, and produce output to nudge the
                // expectation towards the desire
                // TODO: actually drive the actual state using this output
                controller.calculate(nextXhat, setpoint);

                // combine LQR and FF
                Matrix<N1, N1> u = controller.getU();
                Matrix<N1, N1> uff = feedforward.calculate(setpoint);
                state.controlU = u.plus(uff).get(0, 0);
            }

            state.predictedPosition = observer.getXhat(0);
            state.predictedVelocity = observer.getXhat(1);
            // the errors should be zero at the *next* filter quantum.
            // note negative error means the data is below the prediction i.e. prediction
            // should be more negative
            state.residualPosition = MathUtil.angleModulus(state.actualPosition - state.predictedPosition);
            state.residualVelocity = state.actualVelocity - state.predictedVelocity;

            printRow(state);
        }
    }

    @Test
    public void testSinusoidal() {
        System.out.println("\n\nSINUSOIDAL");

        printHeader();

        CompleteState state = new CompleteState();

        state.actualTime = 0;

        state.actualPosition = 1;
        state.actualVelocity = 0;
        state.actualAcceleration = -1;

        state.observedPosition = 1;
        state.observedVelocity = 0;
        state.observedAcceleration = -1;

        state.predictedPosition = 1;
        state.predictedVelocity = 0;

        state.residualPosition = 0;
        state.residualVelocity = 0;

        // initial control
        state.controlU = -1;

        final AngleEstimator observer = newObserver(1, 0);
        final LinearSystem<N2, N1, N2> plant = newPlant();
        final LinearPlantInversionFeedforward<N2, N1, N2> feedforward = new LinearPlantInversionFeedforward<>(plant,
                filterTimeQuantum);
        // initial position
        feedforward.calculate(VecBuilder.fill(1, 0));
        final AngleController controller = newController(plant);

        for (state.step = 0; state.step < 2000; ++state.step) {
            state.actualTime = state.step * actualTimeQuantum;

            state.actualAcceleration = -1.0 * Math.cos(state.actualTime);
            state.actualVelocity = -1.0 * Math.sin(state.actualTime);
            state.actualPosition = MathUtil.angleModulus(Math.cos(state.actualTime));

            state.observedVelocity = state.actualVelocity;
            state.observedPosition = state.actualPosition;
            state.observedAcceleration = state.actualAcceleration;

            if (state.step % stepsPerFilterQuantum == 0) {
                // talk to the observer and controller periodically
                // the goal is to decide what to do for the next quantum,
                // so here we are looking at the future.

                // 1. correct the observer with current measurements.
                // right now these measurements represent the current instant
                // (TODO: add measurement delay)
                observer.correctAngle(state.controlU, state.observedPosition);
                observer.correctVelocity(state.controlU, state.observedVelocity);

                // 2. predict the expected future state.
                observer.predictState(state.controlU, filterTimeQuantum);

                // this is the predicted future state given the previous control
                Matrix<N2, N1> nextXhat = observer.getXhat();

                // 3. specify the desired future state.
                // (here we magically apply the actual state)
                double setpointPosition = MathUtil.angleModulus(Math.cos(state.actualTime + filterTimeQuantum));
                double setpointVelocity = -1.0 * Math.sin(state.actualTime + filterTimeQuantum);
                Vector<N2> setpoint = VecBuilder.fill(setpointPosition, setpointVelocity);

                // 4. calculate control output.
                // compare the desired and expected states, and produce output to nudge the
                // expectation towards the desire
                // TODO: actually drive the actual state using this output
                controller.calculate(nextXhat, setpoint);

                // combine LQR and FF
                Matrix<N1, N1> u = controller.getU();
                Matrix<N1, N1> uff = feedforward.calculate(setpoint);
                state.controlU = u.plus(uff).get(0, 0);
            }

            state.predictedPosition = observer.getXhat(0);
            state.predictedVelocity = observer.getXhat(1);
            // the errors should be zero at the *next* filter quantum.
            // note negative error means the data is below the prediction i.e. prediction
            // should be more negative
            state.residualPosition = MathUtil.angleModulus(state.actualPosition - state.predictedPosition);
            state.residualVelocity = state.actualVelocity - state.predictedVelocity;

            printRow(state);
        }
    }

    private static void printHeader() {
        System.out.print("        step,   actualTime, ");
        System.out.print("   actualPos,    actualVel,    actualAcc, ");
        System.out.print(" observedPos,  observedVel,  observedAcc, ");
        System.out.print("predictedPos, predictedVel, ");
        System.out.print(" residualPos,  residualVel, ");
        System.out.print("    controlU\n");
    }

    private static void printRow(CompleteState state) {
        System.out.printf("%12d, %12.3f, ",
                state.step,
                state.actualTime);
        System.out.printf("%12.3f, %12.3f, %12.3f, ",
                state.actualPosition,
                state.actualVelocity,
                state.actualAcceleration);
        System.out.printf("%12.3f, %12.3f, %12.3f, ",
                state.observedPosition,
                state.observedVelocity,
                state.observedAcceleration);
        System.out.printf("%12.3f, %12.3f, ",
                state.predictedPosition,
                state.predictedVelocity);
        System.out.printf("%12.3f, %12.3f, ",
                state.residualPosition,
                state.residualVelocity);
        System.out.printf("%12.3f\n",
                state.controlU);
    }

}
