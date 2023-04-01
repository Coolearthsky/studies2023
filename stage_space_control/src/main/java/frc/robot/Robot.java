// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private final LinearSystem<N1, N1, N1> m_flywheelPlant = LinearSystemId.createFlywheelSystem(
      DCMotor.getNEO(2), kFlywheelMomentOfInertia, kFlywheelGearing);
  private static final double kFlywheelMomentOfInertia = 0.00032; // kg * m^2
  private static final double kFlywheelGearing = 1.0;
  private final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
      Nat.N1(),
      Nat.N1(),
      m_flywheelPlant,
      VecBuilder.fill(3.0), // How accurate we think our model is
      VecBuilder.fill(0.01), // How accurate we think our encoder
      // data is
      0.020);
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller = new LinearQuadraticRegulator<>(
      m_flywheelPlant,
      VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
      // this to more heavily penalize state excursion, or make the controller behave
      // more
      // aggressively.
      VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less aggressive. 12
      // is a good
      // starting point because that is the (approximate) maximum voltage of a
      // battery.
      0.020); // Nominal time between loops. 0.02
  private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer,
      12.0, 0.020);
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if (m_joystick.getTriggerPressed()) {
      // We just pressed the trigger, so let's set our next reference
      m_loop.setNextR(VecBuilder.fill(1));
    } else if (m_joystick.getTriggerReleased()) {
      // We just released the trigger, so let's spin down
      m_loop.setNextR(VecBuilder.fill(0.0));
    }

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(m_encoder.getRate()));

    // Update our LQR to generate new voltage commands and use the voltages to
    // predict the next
    // state with out Kalman filter.
    m_loop.predict(0.020);
    
    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    m_motor.setVoltage(nextVoltage);
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
