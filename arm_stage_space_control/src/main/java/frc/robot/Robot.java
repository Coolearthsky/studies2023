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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.system.LinearSystemLoop;

/**
 * This is a sample program to demonstrate how to use a state-space controller
 * to control an arm.
 */
public class Robot extends TimedRobot implements Sendable{
  static double kUpperArmLength = .21; 
  static double kLowerArmLength = .275;
  double[] InvKinOutput;
  double nextVoltage;
  double nextVoltage2;
  double kV;
  double kS;
  double kS2;
  double motorOutput1;
  private CommandXboxController controller0 = new CommandXboxController(1);
  double wrappedPos1;
  double wrappedPos2;
  double posGoal1;
  double posGoal2;
  double motorOutput2;
  private final double kLoweredPosition = Units.degreesToRadians(0.0);

  // Moment of inertia of the arm, in kg * m^2. Can be estimated with CAD. If
  // finding this constant
  // is difficult, LinearSystem.identifyPositionSystem may be better.
  // private static final double kArmMOI = 0.43; //.0861125
  

  // Reduction between motors and encoder, as output over input. If the arm spins
  // slower than
  // the motors, this number should be greater than one.
  private static final double kArmGearing = 45;

  private final TrapezoidProfile.Constraints m_constraints1 = new TrapezoidProfile.Constraints(
      8*Math.PI,
      4*Math.PI); // Max arm speed and acceleration.
      private final TrapezoidProfile.Constraints m_constraints2 = new TrapezoidProfile.Constraints(
        8*Math.PI,
        4*Math.PI);
  private TrapezoidProfile.State m_lastProfiledReference1 = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_lastProfiledReference2 = new TrapezoidProfile.State();

  // The plant holds a state-space model of our arm. This system has the following
  // properties:
  //
  // States: [position, velocity], in radians and radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [position], in radians.
  private final LinearSystem<N2, N1, N1> m_armPlant1 = LinearSystemId
      .createSingleJointedArmSystem(DCMotor.getNEO(1), 0.43, kArmGearing);
      private final LinearSystem<N2, N1, N1> m_armPlant2 = LinearSystemId
      .createSingleJointedArmSystem(DCMotor.getNEO(1), 0.14486725, kArmGearing);

  public Robot() {
    SmartDashboard.putData("Robot", this);
  }
  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N2, N1, N1> m_observer1 = new KalmanFilter<>(
      Nat.N2(),
      Nat.N1(),
      m_armPlant1,
      VecBuilder.fill(0.015, 0.17), // How accurate we
      // think our model is, in radians and radians/sec
      VecBuilder.fill(2*Math.PI/(42*45)), // How accurate we think our encoder position
      // data is. In this case we very highly trust our encoder position reading.
      0.020);
      private final KalmanFilter<N2, N1, N1> m_observer2 = new KalmanFilter<>(
      Nat.N2(),
      Nat.N1(),
      m_armPlant1,
      VecBuilder.fill(0.015, 0.17), // How accurate we
      // think our model is, in radians and radians/sec
      VecBuilder.fill(2*Math.PI/(42*45)), // How accurate we think our encoder position
      // data is. In this case we very highly trust our encoder position reading.
      0.020);

  
  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N2, N1, N1>  m_controller2 = new LinearQuadraticRegulator<>(m_armPlant2, VecBuilder.fill(Units.degreesToRadians(0.01), Units.degreesToRadians(1)), VecBuilder.fill(12),0.02);
  private final LinearQuadraticRegulator<N2, N1, N1>  m_controller1 = new LinearQuadraticRegulator<>(
      m_armPlant1,
      VecBuilder.fill(Units.degreesToRadians(.01), Units.degreesToRadians(1)), // qelms.
      // Position and velocity error tolerances, in radians and radians per second.
      // Decrease
      // this
      // to more heavily penalize state excursion, or make the controller behave more
      // aggressively. In this example we weight position much more highly than
      // velocity, but
      // this
      // can be tuned to balance the two.
      VecBuilder.fill(12), // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less aggressive. 12
      // is a good
      // starting point because that is the (approximate) maximum voltage of a
      // battery.
      0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant
  // for easy control.
  private final LinearSystemLoop<N2, N1, N1> m_loop2 = new LinearSystemLoop<>(m_armPlant2, m_controller2, m_observer2, 12.0, 0.02);
  private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(m_armPlant1, m_controller1, m_observer1, 12.0,
      0.020);

  // An encoder set up to measure arm position in radians.
  private final CANSparkMax m_motor2 = new CANSparkMax(30, MotorType.kBrushless);
  private final CANSparkMax m_motor1 = new CANSparkMax(13, MotorType.kBrushless);
  private final RelativeEncoder encoder1 = m_motor1.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  private final RelativeEncoder encoder2 = m_motor2.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  // A joystick to read the trigger from.
  @Override
  public void robotInit() {
    encoder1.setPosition(0);
    encoder2.setPosition(0);
    // We go 2 pi radians in 1 rotation, or 4096 counts.
  }

  @Override
  public void teleopInit() {
    encoder1.setPosition(0);
    encoder1.setPositionConversionFactor(2*Math.PI/45);
    encoder1.setVelocityConversionFactor(Math.PI/1350);
    encoder1.setMeasurementPeriod(8);
    encoder1.setAverageDepth(1);
    encoder2.setPosition(0);
    encoder2.setPositionConversionFactor(2*Math.PI/45);
    encoder2.setVelocityConversionFactor(Math.PI/1350);
    encoder2.setMeasurementPeriod(8);
    encoder2.setAverageDepth(1);

    m_loop2.reset(VecBuilder.fill(encoder2.getPosition(), encoder2.getVelocity()));
    // Reset our loop to make sure it's in a known state.
    m_loop.reset(
        VecBuilder.fill(encoder1.getPosition(), encoder1.getVelocity()));

    // Reset our last re ference to the current state.
    m_lastProfiledReference1 = new TrapezoidProfile.State(encoder1.getPosition(),
        encoder1.getVelocity());
        m_lastProfiledReference2 = new TrapezoidProfile.State(encoder2.getPosition(),
        encoder2.getVelocity());
  }

  @Override
  public void teleopPeriodic() {
    if (Math.abs(encoder1.getPosition()) > 3){
      m_motor1.setVoltage(0);
      m_motor2.setVoltage(0);
      System.out.println("SAFETY SHUTDOWN");
      return;
    } else {
      System.out.println("Working");
    }



   wrappedPos1 = MathUtil.angleModulus(encoder1.getPosition());
   wrappedPos2 = MathUtil.angleModulus(encoder2.getPosition());
   InvKinOutput = algorithm2RIKS(0, 0);  
   posGoal1 = Math.atan2(MathUtil.applyDeadband(controller0.getLeftX(), 0.05),
        MathUtil.applyDeadband(controller0.getLeftY(), 0.05));
        posGoal1 = MathUtil.angleModulus(InvKinOutput[0]);
        posGoal2 = Math.atan2(MathUtil.applyDeadband(controller0.getRightX(), 0.05), MathUtil.applyDeadband(controller0.getRightY(), 0.05));
        posGoal2 = MathUtil.angleModulus(InvKinOutput[1]);
    // Sets the target position of our arm. This is similar to setting the setpoint
    // of a
    // PID controller.
    TrapezoidProfile.State goal1;
    TrapezoidProfile.State goal2;
    
    if (Math.abs(posGoal2) > 0) {
      // System.out.println("RUNNING MOVE");
      // the trigger is pressed, so we go to the high goal.
      goal2 = new TrapezoidProfile.State(posGoal2, 0.0);
    } else {
      // Otherwise, we go to the low goal
      goal2 = new TrapezoidProfile.State(kLoweredPosition, 0.0);
    }
    if (Math.abs(posGoal1) > 0) {
      // System.out.println("RUNNING MOVE");
      // the trigger is pressed, so we go to the high goal.
      goal1 = new TrapezoidProfile.State(posGoal1, 0.0);
    } else {
      // Otherwise, we go to the low goal
      goal1 = new TrapezoidProfile.State(kLoweredPosition, 0.0);
    }
    // Step our TrapezoidalProfile forward 20ms and set it as our next reference
    m_lastProfiledReference1 = (new TrapezoidProfile(m_constraints1, goal1, m_lastProfiledReference1)).calculate(0.020);

    m_lastProfiledReference2 = (new TrapezoidProfile(m_constraints2, goal2, m_lastProfiledReference2)).calculate(0.020);
    m_loop.setNextR(m_lastProfiledReference1.position, m_lastProfiledReference1.velocity);
    m_loop2.setNextR(m_lastProfiledReference2.position, m_lastProfiledReference2.velocity);

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(encoder1.getPosition()));

    m_loop2.correct(VecBuilder.fill(encoder2.getPosition()));
    // Update our LQR to generate new voltage commands and use xthe voltages to
    // predict the next
    // state with out Kalman filter.
    m_loop2.predict(0.02);
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    nextVoltage = m_loop.getU(0);
    nextVoltage2 = m_loop2.getU(0);

    kS = 0.03 * Math.signum(nextVoltage);
    kS2 = 0.03 * Math.signum(nextVoltage2);
    kV = .28*Math.sin(wrappedPos1);
    motorOutput1 = kV+nextVoltage+kS;
    motorOutput2 = nextVoltage2+kS2;
       m_motor1.setVoltage(motorOutput1);
       m_motor2.setVoltage(motorOutput2);
  }

  static double[] algorithm2RIKS(double x, double y) {
    // https://www.youtube.com/watch?v=RH3iAmMsolo&t=7s

    double[] angles = new double[2];
    double xSquared = Math.pow(x, 2);
    double ySquared = Math.pow(y, 2);

    if (Math.sqrt(ySquared + xSquared) + 0.05 >= kLowerArmLength + kUpperArmLength) {
        return null;
    }

    double upperLengthSquare = Math.pow(kUpperArmLength, 2);
    double lowerLengthSquare = Math.pow(kLowerArmLength, 2);
    double lengthSquared = upperLengthSquare + lowerLengthSquare;
    double q2 = Math.PI
            - Math.acos((lengthSquared - xSquared - ySquared) / (2 * kUpperArmLength * kLowerArmLength));
    // maybe x/y try if it dosent work
    double q1 = Math.atan(y / x)
            - Math.atan((kUpperArmLength * Math.sin(q2)) / (kLowerArmLength + kUpperArmLength * Math.cos(q2)));

    angles[0] = q1 + q2; // upper theta
    angles[1] = q1; // lower theta

    return angles;
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("nextVoltage1", () -> nextVoltage, null);
    builder.addDoubleProperty("wrappedPos1", () -> wrappedPos1, null);
    builder.addDoubleProperty("Velocity 1 (RPM)", () -> encoder1.getVelocity() * 1350/Math.PI, null);
    builder.addDoubleProperty("nextVoltage2", () -> nextVoltage2, null);
    builder.addDoubleProperty("wrappedPos2", () -> wrappedPos2, null);
    builder.addDoubleProperty("Velocity 2 (RPM)", () -> encoder2.getVelocity() * 1350/Math.PI, null);
    builder.addDoubleProperty("kV", () -> kV, null);
    builder.addDoubleProperty("kS", () -> kS, null);
    builder.addDoubleProperty("kS2", () -> kS2, null);
    builder.addDoubleProperty("motorOutput1", () -> motorOutput1, null);
    builder.addDoubleProperty("Position Goal 1 (radians)", () -> posGoal1, null);
    builder.addDoubleProperty("Position Goal 1 (degrees)", () -> Units.radiansToDegrees(posGoal1), null);
    builder.addDoubleProperty("motorOutput2", () -> motorOutput2, null);
    builder.addDoubleProperty("Position Goal 2 (radians)", () -> posGoal2, null);
    builder.addDoubleProperty("Position Goal 2 (degrees)", () -> Units.radiansToDegrees(posGoal2), null);
    // builder.addDoubleArrayProperty("Inv Kin Output", () -> InvKinOutput, null);
  }
}
