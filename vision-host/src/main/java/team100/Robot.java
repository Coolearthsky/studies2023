package team100;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is the simplest possible NetworkTables server,
 * used for figuring out vision encoding.
 */
public class Robot extends TimedRobot {
  private final DoublePublisher timestamp_publisher;

  public Robot() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.startServer("example server");
    NetworkTable table = inst.getTable("example_table");
    timestamp_publisher = table.getDoubleTopic("timestamp").publish();
  }

  @Override
  public void robotPeriodic() {
    timestamp_publisher.set(Timer.getFPGATimestamp());
  }
}
