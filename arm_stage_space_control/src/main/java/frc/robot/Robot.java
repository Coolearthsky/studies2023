// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This is a sample program to demonstrate how to use a state-space controller
 * to control an arm.
 */
public class Robot extends TimedRobot implements Sendable{
  double nextVoltage;
  double kV;
  double kS;
  double motorOutput;
  private CommandXboxController controller0 = new CommandXboxController(1);
  double wrappedPos;
  double setpoint;
  private final double kLoweredPosition = Units.degreesToRadians(0.0);

  // Moment of inertia of the arm, in kg * m^2. Can be estimated with CAD. If
  // finding this constant
  // is difficult, LinearSystem.identifyPositionSystem may be better.
  private static final double kArmMOI = 0.032;

  // Reduction between motors and encoder, as output over input. If the arm spins
  // slower than
  // the motors, this number should be greater than one.
  private static final double kArmGearing = 15;

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
      6*Math.PI,
      6*Math.PI); // Max arm speed and acceleration.
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  // The plant holds a state-space model of our arm. This system has the following
  // properties:
  //
  // States: [position, velocity], in radians and radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [position], in radians.
  private final LinearSystem<N2, N1, N1> m_armPlant = LinearSystemId
      .createSingleJointedArmSystem(DCMotor.getNEO(1), kArmMOI, kArmGearing);

  public Robot() {
    SmartDashboard.putData("Robot", this);
  }
  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
      Nat.N2(),
      Nat.N1(),
      m_armPlant,
      VecBuilder.fill(0.015, 0.17), // How accurate we
      // think our model is, in radians and radians/sec
      VecBuilder.fill(0.01), // How accurate we think our encoder position
      // data is. In this case we very highly trust our encoder position reading.
      0.020);

  
  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
      m_armPlant,
      VecBuilder.fill(Units.degreesToRadians(.1), Units.degreesToRadians(10)), // qelms.
      // Position and velocity error tolerances, in radians and radians per second.
      // Decrease
      // this
      // to more heavily penalize state excursion, or make the controller behave more
      // aggressively. In this example we weight position much more highly than
      // velocity, but
      // this
      // can be tuned to balance the two.
      VecBuilder.fill(20), // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less aggressive. 12
      // is a good
      // starting point because that is the (approximate) maximum voltage of a
      // battery.
      0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant
  // for easy control.
  private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(m_armPlant, m_controller, m_observer, 12.0,
      0.020);

  // An encoder set up to measure arm position in radians.
  private final CANSparkMax m_motor = new CANSparkMax(13, MotorType.kBrushless);
  private final RelativeEncoder encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  // A joystick to read the trigger from.
  @Override
  public void robotInit() {
    // We go 2 pi radians in 1 rotation, or 4096 counts.
  }

  @Override
  public void teleopInit() {
    encoder.setPosition(0);
    encoder.setPositionConversionFactor(2*Math.PI/15);
    encoder.setVelocityConversionFactor(Math.PI/450);

    // Reset our loop to make sure it's in a known state.
    m_loop.reset(
        VecBuilder.fill(wrappedPos, encoder.getVelocity()));

    // Reset our last re ference to the current state.
    m_lastProfiledReference = new TrapezoidProfile.State(wrappedPos,
        encoder.getVelocity());
  }

  @Override
  public void teleopPeriodic() {
   wrappedPos = MathUtil.angleModulus(encoder.getPosition());
    setpoint = Math.atan2(MathUtil.applyDeadband(controller0.getLeftX(), 0.05),
        MathUtil.applyDeadband(controller0.getLeftY(), 0.05));
    // Sets the target position of our arm. This is similar to setting the setpoint
    // of a
    // PID controller.
    TrapezoidProfile.State goal;
    if (Math.abs(setpoint) > 0) {
      System.out.println("RUNNING MOVE");
      // the trigger is pressed, so we go to the high goal.
      goal = new TrapezoidProfile.State(setpoint, 0.0);
    } else {
      // Otherwise, we go to the low goal
      goal = new TrapezoidProfile.State(kLoweredPosition, 0.0);
    }
    // Step our TrapezoidalProfile forward 20ms and set it as our next reference
    m_lastProfiledReference = (new TrapezoidProfile(m_constraints, goal, m_lastProfiledReference)).calculate(0.020);
    m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(wrappedPos));

    // Update our LQR to generate new voltage commands and use xthe voltages to
    // predict the next
    // state with out Kalman filter.
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    nextVoltage = m_loop.getU(0);
    kS = 0.05 * Math.signum(nextVoltage);
    kV = .22*Math.sin(wrappedPos);
    motorOutput = kV+nextVoltage+kS;
    m_motor.setVoltage(motorOutput);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("nextVoltage", () -> nextVoltage, null);
    builder.addDoubleProperty("wrappedPos", () -> wrappedPos, null);
    builder.addDoubleProperty("encoder.getVelocity()", () -> encoder.getVelocity(), null);
    builder.addDoubleProperty("kV", () -> kV, null);
    builder.addDoubleProperty("kS", () -> kS, null);
    builder.addDoubleProperty("motorOutput", () -> motorOutput, null);
    builder.addDoubleProperty("setpoint", () -> setpoint, null);
  }
}
