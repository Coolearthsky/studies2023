package frc.robot.visualization;

import static java.util.Map.entry;

import java.util.Map;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.math.ArmKinematics;
import frc.robot.subsystems.Arm;

/** Observes the arm position asynchronously and tells the dashboard. */
public class ArmVisualization {
  private final Map<Arm.Axis, Double> m_offset = Map.ofEntries(
      entry(Arm.Axis.Swing, 0.0),
      entry(Arm.Axis.Boom, 0.0),
      entry(Arm.Axis.Stick, 0.0),
      entry(Arm.Axis.Wrist, -90.0),
      entry(Arm.Axis.Twist, 0.0),
      entry(Arm.Axis.Grip, 0.0));
  // maps [0,1] to degrees
  private final Map<Arm.Axis, Double> m_scale = Map.ofEntries(
      entry(Arm.Axis.Swing, 180.0),
      entry(Arm.Axis.Boom, 180.0),
      entry(Arm.Axis.Stick, -180.0),
      entry(Arm.Axis.Wrist, 180.0),
      entry(Arm.Axis.Twist, 180.0),
      entry(Arm.Axis.Grip, 180.0));

  private final Arm m_arm;

  private final Mechanism2d m_topView = new Mechanism2d(100, 100);
  private final MechanismRoot2d m_topRoot = m_topView.getRoot("TopRoot", 50, 50);
  private final MechanismLigament2d m_swingLigament = new MechanismLigament2d("Swing", 15, 90, 5,
      new Color8Bit(Color.kWhite));

  private final Mechanism2d m_sideView = new Mechanism2d(100, 100);
  private final MechanismRoot2d m_sideRoot = m_sideView.getRoot("SideRoot", 50, 50);
  private final MechanismLigament2d m_boomLigament = new MechanismLigament2d("Boom",
      ArmKinematics.kBoomLength, 90, 5, new Color8Bit(Color.kWhite));
  private final MechanismLigament2d m_stickLigament = new MechanismLigament2d("Stick",
      ArmKinematics.kStickLength, -90, 5, new Color8Bit(Color.kLightGreen));
  private final MechanismLigament2d m_wristLigament = new MechanismLigament2d("Wrist",
      ArmKinematics.kWristLength, -90, 5, new Color8Bit(Color.kLightBlue));

  private final Notifier m_notifier = new Notifier(this::update);

  /** Visualize the arm position in the dashboard. */
  public ArmVisualization(Arm arm) {
    m_arm = arm;
    m_topRoot.append(m_swingLigament);
    SmartDashboard.putData("TopView", m_topView);
    m_sideRoot.append(m_boomLigament);
    m_boomLigament.append(m_stickLigament);
    m_stickLigament.append(m_wristLigament);
    SmartDashboard.putData("SideView", m_sideView);
  }

  /* Starts the notifier. */
  public void start() {
    m_notifier.startPeriodic(0.1); // 10Hz = slower than the robot loop
  }

  /* Reads current servo positions and writes them to the dashboard */
  public void update() {
    double boomAngle = getMechanismAngle(Arm.Axis.Boom);
    double stickAngle = getMechanismAngle(Arm.Axis.Stick);
    double swingAngle = getMechanismAngle(Arm.Axis.Swing);
    double swingLength = ArmKinematics.kBoomLength * Math.cos(Math.toRadians(boomAngle))
        + ArmKinematics.kStickLength * Math.cos(Math.toRadians(boomAngle + stickAngle));
    double wristAngle = getMechanismAngle(Arm.Axis.Wrist);

    m_swingLigament.setAngle(swingAngle);
    m_swingLigament.setLength(swingLength);
    m_boomLigament.setAngle(boomAngle);
    m_stickLigament.setAngle(stickAngle);
    m_wristLigament.setAngle(wristAngle);
  }

  private double getMechanismAngle(Arm.Axis axis) {
    return m_arm.getPosition(axis) * m_scale.get(axis) + m_offset.get(axis);
  }
}
