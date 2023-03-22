package frc.robot.subsystems.DistanceSensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class GP2Y0A21 extends DistanceSensor {
    private final AnalogInput sensor;
    
    private double surfaceOffset;
    private double distance;

    public GP2Y0A21(int analogPort, double surfaceOffsetMillimeters) {
        sensor = new AnalogInput(analogPort);
        sensor.setOversampleBits(7);
        this.surfaceOffset = surfaceOffsetMillimeters;
    }

    public GP2Y0A21(int analogPort) {
        this(analogPort, 0);
    }

    @Override
    public void periodic() {
        distance = readSensor() * 1.65; // Seemed to give more accurate data, may need further adjustments.
                                        // Possibly because these sensors are not as accurate at close ranges
                                        // (They are only rated for 10-80cm, and I need them for 0.1-18cm)
    }

    private double readSensor() {
        return (
            // I measured the voltage based on certain distances and
            // fit an exponential function to it:
            // https://www.desmos.com/calculator/rpzuxriawt
            Math.exp(-1.52 * (sensor.getAverageVoltage() - 4.32))
            + 78.7
        );
    }

    public double getMillimeters() {
        return distance - surfaceOffset;
    }

    public double getCentimeters() {
        return getMillimeters() / 10.0;
    }
}