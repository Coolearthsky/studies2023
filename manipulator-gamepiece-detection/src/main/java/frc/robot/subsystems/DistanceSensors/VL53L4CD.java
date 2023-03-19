package frc.robot.subsystems.DistanceSensors;

import java.util.concurrent.TimeoutException;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;

/**
 * A class for interfacing with the VL53L4CD distance sensor.
 * <p>Based on the VL53L4CD CircuitPython library by Adafruit</p>
 * <p>
 * <a href="https://github.com/adafruit/Adafruit_CircuitPython_VL53L4CD/blob/main/adafruit_vl53l4cd.py">Adafruit VL53L4CD CircuitPython Library</a>
 */
public class VL53L4CD extends DistanceSensor {
    private double distance; // the distance in millimeters
    private double surfaceOffset;

    private State state;
    
    private I2C i2c;
    private DigitalOutput enable;

    private static final class Registers {
        static final int I2C_SLAVE_DEVICE_ADDRESS = 0x0001;
        static final int GPIO_HV_MUX_CTRL = 0x0030;
        static final int GPIO_TIO_HV_STATUS = 0x0031;
        static final int SYSTEM_START = 0x0087;
        static final int RESULT_DISTANCE = 0x0096;
    }

    private enum State {
        BOOTING,
        IDLE,
        RANGING,
        ERROR;
    }
    
    /**
     * Creates a new VL53L4CD distance sensor.
     * @param address the I2C address of the sensor (default is 0x52)
     * @param surfaceOffsetMillimeters the offset to apply to the distance measurement
     *                                 (e.g. if the sensor is mounted behind an inside surface, 
     *                                 this should be the distance from the sensor to the surface)
     */
    public VL53L4CD(int address, int enablePin, double surfaceOffsetMillimeters) {
        i2c = new I2C(I2C.Port.kMXP, 0x29);
        enable = new DigitalOutput(enablePin);

        surfaceOffset = surfaceOffsetMillimeters;
        state = State.BOOTING;
        distance = 0;
        
        configure(address);
    }

    /**
     * Configures the sensor to use the given address and waits for it to boot.
     * @param address
     */
    private void configure(int address) {
        enable.set(true);
        i2c.write(Registers.I2C_SLAVE_DEVICE_ADDRESS, address);
        i2c = new I2C(I2C.Port.kMXP, address);
        try {
            waitForBoot();
        } catch (TimeoutException e) {
            e.printStackTrace();
            enable.set(false);
        }
    }

    private void waitForBoot() throws TimeoutException {
        for (int i = 0; i < 1000; i++) {
            if (isDataReady()) {
                state = State.IDLE;
                return;
            }
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        state = State.ERROR;
        throw new TimeoutException("Sensor did not boot in time");
    }

    private byte[] readRegister(int register, int length) {
        byte[] data = new byte[length];
        i2c.read(register, length, data);
        return data;
    }

    private int getInterruptPolarity() {
        int polarity = readRegister(Registers.GPIO_HV_MUX_CTRL, 1)[0] & 0x10;
        polarity = (polarity >> 4) & 0x01;
        return polarity == 0 ? 1 : 0;
    }

    private boolean isDataReady() {
        byte[] status = readRegister(Registers.GPIO_TIO_HV_STATUS, 1);
        return (status[0] & 0x01) == getInterruptPolarity();
    }

    private void startRanging() {
        i2c.write(Registers.SYSTEM_START, 0x40);
        state = State.RANGING;
    }

    private double getDistance() {
        byte[] data = readRegister(Registers.RESULT_DISTANCE, 2);
        int dist = (data[0] << 8) | data[1];
        return dist / 10.0;
    }

    public void periodic() {
        if (state == State.IDLE) {
            startRanging();
        }

        if (state == State.RANGING && isDataReady()) {
            distance = getDistance();
        }
    }

    /**
     * Returns the latest distance measurement from the sensor.
     * @return the distance in millimeters
     */
    public double getMillimeters() {
        return distance - surfaceOffset;
    }
}
