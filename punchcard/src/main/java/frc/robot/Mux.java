package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * Wraps the Sparkfun 16-channel analog Multiplexer based on the Texas
 * Instruments CD74HC4067.
 * 
 * https://www.sparkfun.com/products/9056
 */
public class Mux {
    private static final int kSelectors = 4;
    private final DigitalOutput[] m_outputs;

    /**
     * @param outputs the digital IO pins corresponding to each selector pin.
     */
    public Mux(DigitalOutput[] outputs) {
        if (outputs.length != 4)
            throw new IllegalArgumentException("must supply four channels. you supplied " + outputs.length);

        m_outputs = outputs;
    }

    public void set(int channel) {
        boolean[] truth = truth(channel);
        for (int i = 0; i < kSelectors; ++i) {
            m_outputs[i].set(truth[i]);
        }
    }

    /**
     * The truth table as described on the datasheet page two:
     * https://www.sparkfun.com/datasheets/IC/cd74hc4067.pdf
     */
    static boolean[] truth(int channel) {
        return Util.toArray(channel, kSelectors);
    }

}
