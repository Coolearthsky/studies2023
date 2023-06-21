package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot implements Sendable {

  AHRS ahrs_i2c;
  AHRS ahrs_usb;
  float gyroZOffset_I2C;
  float gyroZOffset_USB;

  public Robot() {
    ahrs_i2c = new AHRS(I2C.Port.kMXP);
    ahrs_usb = new AHRS(SerialPort.Port.kUSB1);
    SmartDashboard.putData("demo", this);

    ahrs_i2c.enableBoardlevelYawReset(true);
    ahrs_usb.enableBoardlevelYawReset(true);
    ahrs_i2c.calibrate();
    ahrs_usb.calibrate();
    while (ahrs_i2c.isCalibrating() || ahrs_usb.isCalibrating()){
      System.out.println("Waiting for calibration to finish");
    }
    ahrs_i2c.zeroYaw();
    ahrs_usb.zeroYaw();
    gyroZOffset_I2C = -ahrs_i2c.getRawGyroZ();
    gyroZOffset_USB = -ahrs_usb.getRawGyroZ();
  }

  @Override
  public void robotPeriodic() {
    //System.out.println(Timer.getFPGATimestamp());
   // System.out.println(ahrs.getYaw());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("ahrs_usb yaw", () -> ahrs_usb.getYaw(), null);
    builder.addDoubleProperty("ahrs_usb Update Rate", () -> ahrs_usb.getActualUpdateRate(), null);
    builder.addDoubleProperty("ahrs_usb Count", () -> ahrs_usb.getUpdateCount(), null);
    builder.addBooleanProperty("ahrs_usb Connected", () -> ahrs_usb.isConnected(), null);
    builder.addDoubleProperty("ahrs_usb TimeStamp", () -> ahrs_usb.getLastSensorTimestamp(), null);
    builder.addDoubleProperty("ahrs_i2c yaw", () -> ahrs_i2c.getYaw(), null);
    builder.addDoubleProperty("ahrs_i2c Update Rate", () -> ahrs_i2c.getActualUpdateRate(), null);
    builder.addDoubleProperty("ahrs_i2c Count", () -> ahrs_i2c.getUpdateCount(), null);
    builder.addBooleanProperty("ahrs_i2c Connected", () -> ahrs_i2c.isConnected(), null);
    builder.addDoubleProperty("ahrs_i2c TimeStamp", () -> ahrs_i2c.getLastSensorTimestamp(), null);
    builder.addDoubleProperty("ahrs_i2c Rate", () -> ahrs_i2c.getRate(), null);
    builder.addDoubleProperty("ahrs_usb Rate", () -> ahrs_usb.getRate(), null);
    builder.addDoubleProperty("ahrs_i2c RawGyro_Z", () -> (ahrs_i2c.getRawGyroZ()+gyroZOffset_I2C), null);
    builder.addDoubleProperty("ahrs_usb RawGyro_Z", () -> (ahrs_usb.getRawGyroZ()+gyroZOffset_USB), null);
    builder.addDoubleProperty("Redundant GyroZ", () -> getRedundantGyroZ(), null);
    builder.addDoubleProperty("Redundant Yaw", () -> getRedundantYaw(), null);
    builder.addDoubleProperty("Redundant Rate", () -> getRedundantRate(), null);
    
  }

  public float getRedundantYaw() {
    float redundYaw = 0;
    int tmpInputs = 0;
    if (ahrs_i2c.isConnected()) {
      redundYaw += ahrs_i2c.getYaw();
      tmpInputs += 1;
    }
    if (ahrs_usb.isConnected()) {
      redundYaw += ahrs_usb.getYaw();
      tmpInputs +=1;
    }
    return (redundYaw)/tmpInputs;
  }

  public float getRedundantRate() {
    float redundRate = 0;
    int tmpInputs = 0;
    if (ahrs_i2c.isConnected()) {
      redundRate += ahrs_i2c.getRate();
      tmpInputs += 1;
    }
    if (ahrs_usb.isConnected()) {
      redundRate += ahrs_usb.getRate();
      tmpInputs +=1;
    }
    return (redundRate)/tmpInputs;
  }

  public float getRedundantGyroZ() {
    float redundGyroZ = 0;
    int tmpInputs = 0;
    if (ahrs_i2c.isConnected()) {
      redundGyroZ += ahrs_i2c.getRawGyroZ() + gyroZOffset_I2C;
      tmpInputs += 1;
    }
    if (ahrs_usb.isConnected()) {
      redundGyroZ += ahrs_usb.getRawGyroZ() + gyroZOffset_USB;
      tmpInputs +=1;
    }
    return (redundGyroZ)/tmpInputs;
  }

}
  
