package org.team100.lib.barcode;

import java.util.OptionalInt;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;

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
 * pins which activate the "odd" and "even" emitters. These are connected to DIO
 * outputs, and can be left "off" if the value isn't being read.
 * 
 * Note that the array uses *one based* numbering for the sensors and for the
 * meaning of "odd" and "even," but the sensor pins themselves are wired in a
 * zero-based way.
 * 
 * The LED controller appears to be an On Semi FAN5622 series, which responds
 * to the CTRL signal in about 85 us to turn on, and 600us to turn off,
 * which is fast but not that fast; we should read all the odds, flip the
 * emitters,
 * and then read all the evens.
 * 
 * https://www.onsemi.com/pdf/datasheet/fan5626-d.pdf
 * 
 * The photosensor on/off threshold also varies by position to account for light
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
 * Note that the response time of the sensor depends on the pull-up resistor
 * value, and the Pololu board uses quite a large resistor, 47k ohms, which
 * means that
 * the rise and fall times are about 300 us.
 * 
 * Datasheet:
 * 
 * https://components.omron.com/us-en/datasheet_pdf/E602-E1.pdf
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
    private final DigitalOutput m_odd;
    private final DigitalOutput m_even;

    /**
     * @param input      all the sensors are multiplexed to one analog input.
     * @param thresholds for each sensor, N of them.
     * @param mux        the array uses N pins starting at zero.
     * @param odd        turn on the odd emitters when high
     * @param even       turn on the even emitters when high
     */
    public Sensor(
            AnalogInput input,
            double[] thresholds,
            Mux mux,
            DigitalOutput odd,
            DigitalOutput even) {
        m_input = input;
        m_thresholds = thresholds;
        m_mux = mux;
        m_odd = odd;
        m_even = even;
        // the default state for the outputs should be off
        m_odd.set(false);
        m_even.set(false);
    }

    /** @return the number represented by the barcode. */
    public OptionalInt readValue() {
        boolean[] read = read();
        if (read.length == 0)
            return OptionalInt.empty();
        return OptionalInt.of(toChannel(read));
    }

    /////////////////////////////////////////////////

    /**
     * @return array representing the state of the sensors, 1 means dark
     */
    boolean[] read() {
        try {
            boolean[] result = new boolean[m_thresholds.length];
            // to read all the bits, we have to scan with the mux.
            // first turn on the odd emitters, which actually means 0,2,4.
            m_odd.set(true);
            Thread.sleep(0, 85);// led driver startup time
            Thread.sleep(0, 200);// sensor startup time minus the mux time below
            for (int i = 0; i < m_thresholds.length; i += 2) {
                m_mux.set(i);
                Thread.sleep(0, 100);// 100ns of settling time
                result[i] = readInput(m_thresholds[i]);
            }
            // switch the emitters to 1,3 (note the confusing naming here)
            m_odd.set(false);
            m_even.set(true);
            Thread.sleep(0, 85);// led driver startup time
            Thread.sleep(0, 200);// sensor startup time minus the mux time below
            for (int i = 1; i < m_thresholds.length; i += 2) {
                m_mux.set(i);
                Thread.sleep(0, 100);// 100ns of settling time
                result[i] = readInput(m_thresholds[i]);
            }
            m_even.set(false);
            return result;
        } catch (InterruptedException e) {
            e.printStackTrace();
            return new boolean[0]; // empty means failure
        } finally {
            // make sure the emitters are off
            m_odd.set(false);
            m_even.set(false);
        }
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
