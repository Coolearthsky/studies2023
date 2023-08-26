// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.commands.HomeCommand;
import frc.robot.subsystems.LaundryArm;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final Talon m_leftMotor = new Talon(0);
  private final Talon m_rightMotor = new Talon(1);
  private final CANSparkMax m_armMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Constraints m_constraints = new Constraints(5, 2);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(.1, .1, .1, m_constraints);
  private final DigitalInput m_upperSwitch = new DigitalInput(3);
  private final DigitalInput m_lowerSwitch = new DigitalInput(4);
  private final LaundryArm m_arm = new LaundryArm(m_controller, m_armMotor, m_upperSwitch, m_lowerSwitch);
  private final HomeCommand m_home = new HomeCommand(m_arm);
  private final Joystick m_stick = new Joystick(0);

  @Override
  public void robotInit() {
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_stick.getY(), -m_stick.getX(), false);
  }
}
