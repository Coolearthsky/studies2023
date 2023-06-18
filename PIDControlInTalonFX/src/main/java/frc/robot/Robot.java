// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //The can ID of the motor controller
  private final int canID = 11;
  //I used a TalonFX for this test change if you have a different motor controller use that instead
  WPI_TalonFX m_motor = new WPI_TalonFX(canID);
  private RobotContainer m_robotContainer; 

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_motor.configFactoryDefault();
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_motor.setSensorPhase(true);
    m_motor.configNominalOutputForward(0);
		m_motor.configNominalOutputReverse(0);
		m_motor.configPeakOutputForward(1);
		m_motor.configPeakOutputReverse(-1);
    m_motor.configAllowableClosedloopError(0, 0, 30);
    //F is feedforward, none used for this test
    m_motor.config_kF(0, 0);
    //The P value you have for position control changes, but 0.25 was good for me. I also found having a D value helped.
    //The P value for velocity seems to affect how fast the motor will spin, so if the P value is smaller, the motor will go slower by a multiplier of how much lower the P value is. My P value I used was 0.0001. The lower P value allows the motor to not go crazy, this allows for much more control.
    m_motor.config_kP(0, 0.0001); // P 0.25
    m_motor.config_kI(0, 0); // P 0
    m_motor.config_kD(0, 0); // P 20
    // int absolutePosition = 0;
    // m_motor.setSelectedSensorPosition(absolutePosition, 0, 30);
  }

  @Override
  public void 
  robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override 
  public void teleopPeriodic() {
    System.out.println(m_motor.get());
    //The gear ratio of the motor, of the motor spins less than the device this value should be greater than one
    double motorGearing = 6.6;
    //There is a set value for the velocity that depends on the P value of the pid controller P Constant = 10/P Value
    double speedConstant = 100000;
    // Units are rots per sec
    double desiredVelocity = 1;
    m_motor.set(ControlMode.Velocity, desiredVelocity * speedConstant * motorGearing);
    //For the talon FX encoder there were 2048 rots per rev, please check to see what yours is.
    int rotationsPerRev = 2048;
    //Units are rotations
    double desiredPosition = 1;
    // m_motor.set(ControlMode.Position, desiredPosition * rotationsPerRev * motorGearing);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {}
}
