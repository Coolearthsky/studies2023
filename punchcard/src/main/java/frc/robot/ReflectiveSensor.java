package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Represents a single Omron EE-SY1201 Reflective Photomicrosensor.
 * 
 * Datasheet:
 * 
 * https://components.omron.com/eu-en/sites/components.omron.com.eu/files/datasheet_pdf/E602-E1.pdf
 * 
 * 
 * The output is inversely proportional to measures incident light: 0v is a lot,
 * 5v is dark. so ambient light can drive it low and if the surface is too
 * close, then the illuminator is shaded from the sensor, so a "light" surface
 * can seem "dark" so you need the surface to be a few mm away.
 * 
 * These sensors are sensitive to ambient light, some of which is emitted by
 * their neighbors in an array, so the on/off thresholds vary.
 */
public class ReflectiveSensor {
    private final AnalogInput m_input;
    private final double m_threshold;

    /**
     * @param input     is injected here so all the port-allocating stuff happens
     *                  somewhere else.
     * @param threshold above the threshold, dark, is 1, below, light, is 0.
     */
    public ReflectiveSensor(AnalogInput input, double threshold) {
        m_input = input;
        m_threshold = threshold;
    }

    /** true = dark. */
    public boolean read() {
        return m_input.getAverageVoltage() > m_threshold;
    }

}
