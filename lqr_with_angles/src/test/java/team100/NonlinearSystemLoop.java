package team100;


import java.util.function.Function;

import org.ejml.MatrixDimensionException;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

/**
 * A copy of WPILib LinearSystemLoop that works with AngleEKF.
 */
public class NonlinearSystemLoop{
  private final AngleController m_controller;
  private final LinearPlantInversionFeedforward<N2, N1, N2> m_feedforward;
  private final AngleEstimator m_observer;
  private Matrix<N2, N1> m_nextR;
  private Function<Matrix<N1, N1>, Matrix<N1, N1>> m_clampFunction;

  /**
   * Constructs a state-space loop with the given plant, controller, and observer. By default, the
   * initial reference is all zeros. Users should call reset with the initial system state before
   * enabling the loop. This constructor assumes that the input(s) to this system are voltage.
   *
   * @param plant State-space plant.
   * @param controller State-space controller.
   * @param observer State-space observer.
   * @param maxVoltageVolts The maximum voltage that can be applied. Commonly 12.
   * @param dtSeconds The nominal timestep.
   */
  public NonlinearSystemLoop(
      LinearSystem<N2, N1, N2> plant,
      AngleController controller,
      AngleEstimator observer,
      double maxVoltageVolts,
      double dtSeconds) {
    this(
        controller,
        new LinearPlantInversionFeedforward<>(plant, dtSeconds),
        observer,
        u -> StateSpaceUtil.desaturateInputVector(u, maxVoltageVolts));
  }

  /**
   * Constructs a state-space loop with the given controller, feedforward, and observer. By default,
   * the initial reference is all zeros. Users should call reset with the initial system state
   * before enabling the loop.
   *
   * @param controller State-space controller.
   * @param feedforward Plant inversion feedforward.
   * @param observer State-space observer.
   * @param clampFunction The function used to clamp the input U.
   */
  public NonlinearSystemLoop(
      AngleController controller,
      LinearPlantInversionFeedforward<N2, N1, N2> feedforward,
      AngleEstimator observer,
      Function<Matrix<N1, N1>, Matrix<N1, N1>> clampFunction) {
    this.m_controller = controller;
    this.m_feedforward = feedforward;
    this.m_observer = observer;
    this.m_clampFunction = clampFunction;

    m_nextR = new Matrix<>(new SimpleMatrix(controller.getK().getNumCols(), 1));
    reset(m_nextR);
  }

  /**
   * Returns an element of the observer's state estimate x-hat.
   *
   * @param row Row of x-hat.
   * @return the i-th element of the observer's state estimate x-hat.
   */
  public double getXHat(int row) {
    return getObserver().getXhat(row);
  }

  /**
   * Set the next reference r.
   *
   * @param nextR Next reference.
   */
  public void setNextR(Matrix<N2, N1> nextR) {
    m_nextR = nextR;
  }

  /**
   * Set the next reference r.
   *
   * @param nextR Next reference.
   */
  public void setNextR(double... nextR) {
    if (nextR.length != m_nextR.getNumRows()) {
      throw new MatrixDimensionException(
          String.format(
              "The next reference does not have the "
                  + "correct number of entries! Expected %s, but got %s.",
              m_nextR.getNumRows(), nextR.length));
    }
    m_nextR = new Matrix<>(new SimpleMatrix(m_nextR.getNumRows(), 1, true, nextR));
  }

  /**
   * Returns the controller's calculated control input u plus the calculated feedforward u_ff.
   *
   * @return the calculated control input u.
   */
  public Matrix<N1, N1> getU() {
    return clampInput(m_controller.getU().plus(m_feedforward.getUff()));
  }

  /**
   * Returns an element of the controller's calculated control input u.
   *
   * @param row Row of u.
   * @return the calculated control input u at the row i.
   */
  public double getU(int row) {
    return getU().get(row, 0);
  }

  /**
   * Return the controller used internally.
   *
   * @return the controller used internally.
   */
  public AngleController getController() {
    return m_controller;
  }

  /**
   * Return the feedforward used internally.
   *
   * @return the feedforward used internally.
   */
  public LinearPlantInversionFeedforward<N2, N1, N2> getFeedforward() {
    return m_feedforward;
  }

  /**
   * Return the observer used internally.
   *
   * @return the observer used internally.
   */
  public AngleEstimator getObserver() {
    return m_observer;
  }

  /**
   * Zeroes reference r and controller output u. The previous reference of the
   * PlantInversionFeedforward and the initial state estimate of the KalmanFilter are set to the
   * initial state provided.
   *
   * @param initialState The initial state.
   */
  public void reset(Matrix<N2, N1> initialState) {
    m_nextR.fill(0.0);
    m_controller.reset();
    m_feedforward.reset(initialState);
    m_observer.setXhat(initialState);
  }

  /**
   * Correct the state estimate x-hat using the measurements in y.
   *
   * @param y Measurement
   */
  public void correctAngle(double y) {
    getObserver().correctAngle(getU().get(0,0), y);
  }

  public void correctVelocity(double y) {
    getObserver().correctVelocity(getU().get(0,0), y);
  }

  /**
   * Sets new controller output, projects model forward, and runs observer prediction.
   *
   * <p>After calling this, the user should send the elements of u to the actuators.
   *
   * @param dtSeconds Timestep for model update.
   */
  public void predict(double dtSeconds) {
    var u =
        clampInput(
            m_controller
                .calculate(getObserver().getXhat(), m_nextR)
                .plus(m_feedforward.calculate(m_nextR)));
    getObserver().predictState(u.get(0,0), dtSeconds);
  }

  /**
   * Clamp the input u to the min and max.
   *
   * @param unclampedU The input to clamp.
   * @return The clamped input.
   */
  public Matrix<N1, N1> clampInput(Matrix<N1, N1> unclampedU) {
    return m_clampFunction.apply(unclampedU);
  }
}
