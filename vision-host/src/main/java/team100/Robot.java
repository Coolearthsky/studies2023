package team100;

import java.io.IOException;
import java.util.EnumSet;

import org.msgpack.jackson.dataformat.MessagePackFactory;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is the simplest possible NetworkTables server,
 * used for figuring out vision encoding.
 */
public class Robot extends TimedRobot {
  private final DoublePublisher timestamp_publisher;

  private final ObjectMapper object_mapper;

  public Robot() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.startServer("example server");
    NetworkTable example_table = inst.getTable("example_table");
    timestamp_publisher = example_table.getDoubleTopic("timestamp").publish();
    object_mapper = new ObjectMapper(new MessagePackFactory());
    NetworkTable vision_table = inst.getTable("Vision");
    inst.addListener(
        vision_table.getEntry("tags"),
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        (event) -> accept(event));
  }

  private void accept(NetworkTableEvent event) {
    try {
      Blips blips = object_mapper.readValue(event.valueData.value.getRaw(), Blips.class);
      System.out.printf("PAYLOAD %s\n", blips);
      System.out.printf("DELAY (s): %f\n", blips.et);
      System.out.printf("BLIP COUNT: %d\n", blips.tags.size());
      for (Blip b: blips.tags ) {
        Pose3d p = blipToPose(b);
        System.out.printf("TAG ID: %d\n", b.id);
        System.out.printf("POSE: %s\n", p);
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private Pose3d blipToPose(Blip b) {
    Translation3d t = new Translation3d(b.pose_t[0][0], b.pose_t[1][0], b.pose_t[2][0]);
    Matrix<N3, N3> rot = new Matrix<N3, N3>(Nat.N3(), Nat.N3());
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        rot.set(i,j,b.pose_R[i][j]);
      }
    }
    Rotation3d r = new Rotation3d(rot);
    Pose3d p = new Pose3d(t, r);
    return p;
  }

  @Override
  public void robotPeriodic() {
    timestamp_publisher.set(Timer.getFPGATimestamp());
  }
}
