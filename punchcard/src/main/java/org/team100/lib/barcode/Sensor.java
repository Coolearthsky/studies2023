package org.team100.lib.barcode;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Represents the id sensor assembly, which consists of two parts:
 * 
 * * Pololu QTRX-HD-05A, a 5-channel reflective infrared photosensor
 * * Sparkfun 16-channel multiplexer
 * 
 * The sensor provides 5 outputs, which are multiplexed to one RoboRIO analog
 * input.
 * 
 * The multiplexer is controlled with 4 inputs, which are driven by RoboRIO
 * digital outputs. The same multiplexer (and analog input) may be used for
 * other inputs, e.g. a second photosensor, dip switches, etc.
 * 
 * The sensor emitters leak light to their neighbors, so there are two control
 * pins which activate the odd and even emitters. These are connected to DIO
 * outputs. The on/off threshold also varies by position to account for light
 * leakage.
 * 
 * The sensor used seems to be the Omron EE-SY1201 Reflective Photomicrosensor.
 * 
 * The output is a resistor pulling up and a photocell pulling down, so the
 * output is inversely proportional to measures incident light: 0v is a lot,
 * 5v is dark. so ambient light can drive it low and if the surface is too
 * close, then the illuminator is shaded from the sensor, so a "light" surface
 * can seem "dark" so you need the surface to be a few mm away -- actually the
 * focal length is 3mm, so that's where the surface should be too.
 * 
 * Datasheet:
 * 
 * https://components.omron.com/eu-en/sites/components.omron.com.eu/files/datasheet_pdf/E602-E1.pdf
 * 
 * 
 * Pololu product page:
 * 
 * https://www.pololu.com/product/4405/resources
 */
public class Sensor {
    private final double[] m_thresholds;
    private final AnalogInput m_input;
    private final Mux m_mux;

    /**
     * @param input      all the sensors are multiplexed to one analog input.
     * @param thresholds for each sensor, N of them.
     * @param mux        the array uses N pins starting at zero.
     */
    public Sensor(AnalogInput input, double[] thresholds, Mux mux) {
        m_input = input;
        m_thresholds = thresholds;
        m_mux = mux;
    }

    /** @return the number represented by the barcode. */
    public int readValue() {
        return toChannel(read());
    }

    /////////////////////////////////////////////////

    /**
     * @return array representing the state of the sensors, 1 means dark
     */
    boolean[] read() {
        // to read all the bits, we have to scan with the mux.
        boolean[] result = new boolean[m_thresholds.length];
        for (int i = 0; i < m_thresholds.length; ++i) {
            m_mux.set(i);
            try {
                Thread.sleep(0, 100);// 100ns of settling time
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            result[i] = readInput(m_thresholds[i]);
        }
        return result;
    }

    /**
     * @param threshold above the threshold means dark, 1, below means light, is 0
     * @return true = dark = "1" in the bar code.
     */
    boolean readInput(double threshold) {
        return m_input.getAverageVoltage() > threshold;
    }

    /** @return int representation of array */
    static int toChannel(boolean[] array) {
        int result = 0;
        for (int i = 0; i < array.length; ++i) {
            result = result | ((array[i] ? 1 : 0) << i);
        }
        return result;
    }
}
