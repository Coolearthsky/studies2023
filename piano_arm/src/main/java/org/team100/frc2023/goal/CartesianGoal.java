package org.team100.frc2023.goal;


import java.util.function.Supplier;

import org.team100.frc2023.kinematics.LynxArmAngles;
import org.team100.frc2023.math.LynxArmKinematics;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Accepts cartesian input, supplies joint positions. */
public class CartesianGoal implements Sendable, Supplier<LynxArmAngles> {
  private final LynxArmKinematics m_kinematics;
  private double m_x;
  private double m_y;
  private double m_z;

  public CartesianGoal(LynxArmKinematics kinematics) {
    m_kinematics = kinematics;
    SmartDashboard.putData("cartesian goal_reader", this);

  }

  @Override
  public LynxArmAngles get() {
    return m_kinematics.inverse(new Translation3d(m_x, m_y, m_z), 0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("cartesian goal_input");
    builder.addDoubleProperty("X", () -> m_x, (x) -> m_x = x);
    builder.addDoubleProperty("Y", () -> m_y, (y) -> m_y = y);
    builder.addDoubleProperty("Z", () -> m_z, (z) -> m_z = z);
  }
}
