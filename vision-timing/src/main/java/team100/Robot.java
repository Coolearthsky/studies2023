package team100;

import java.util.EnumSet;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * Flashes an LED output which is viewed by the camera.
 * Light another LED based on the camera output.
 */
public class Robot extends TimedRobot {
  private final DigitalOutput led = new DigitalOutput(0);
  private final DigitalOutput seen = new DigitalOutput(1);

  public Robot() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.startServer("example server");
    inst.addListener(
        inst.getTable("Vision").getEntry("led"),
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        (event) -> seen.set(event.valueData.value.getBoolean()));
  }

  @Override
  public void robotPeriodic() {
    led.set((int) (10 * Timer.getFPGATimestamp()) % 10 == 0); // 10% duty cycle 1hz
  }
}
