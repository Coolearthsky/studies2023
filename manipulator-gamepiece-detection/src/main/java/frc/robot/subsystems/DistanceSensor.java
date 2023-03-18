package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;

public class DistanceSensor {
    private boolean waiting; // whether or not the sensor is waiting for a measurement
    private double distance; // the distance in millimeters
    private double surfaceOffset;
    
    private I2C i2c;
    
    public DistanceSensor(int address, double surfaceOffset) {
        i2c = new I2C(I2C.Port.kMXP, address);
        distance = 0;
        waiting = false;
        this.surfaceOffset = surfaceOffset;
    }

    public DistanceSensor() {
        this(0x52, 0);
    }

    private enum Status {
        DATA_READY(0x02),
        MEASUREMENT_TAKEN(0x01);

        int flag;

        Status(int flag) {
            this.flag = flag;
        }

        static int with(Status ...values) {
            int result = 0;
            for (Status value : values) {
                result |= value.flag;
            }
            return result;
        }

        static boolean and(int value, Status ...values) {
            return (value & with(values)) == with(values);
        }
    }

    public void periodic() {
        if (!waiting) {
            // write to the sensor to request a measurement
            i2c.write(0x0180, 0x01);
            waiting = true;
        }

        byte[] status = new byte[1];
        i2c.read(0x14, 1, status);
        if (Status.and(status[0], Status.DATA_READY)) {
            // read the measurement from the sensor
            byte[] measurementData = new byte[17];
            i2c.read(0x62, 17, measurementData);
            distance = ((measurementData[0] << 8) | (measurementData[1] & 0xff)) / 10.0;
            waiting = false;
        }
    }

    public double getMillimeters() {
        return distance - surfaceOffset;
    }
}
