package team100;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

/**
 * Example LED illuminator using a triple Cree XP-E2 from LED Supply, driven
 * with a REV SparkMax controller.
 */
public class Illuminator {
    /**
     * There is no current-limiting resistor in this setup, we rely on voltage to
     * choose an output level; this has the advantage of simplicity and avoiding a
     * multi-watt resistor, but it has the disadvantage that the LED output is not
     * constant with voltage, it depends on temperature in a thermal-runaway
     * fashion. See https://assets.cree-led.com/a/ds/x/XLamp-XPE2.pdf page 3:
     * temperature coefficient of green is -1.2 mV/degC, red is -1.8 mV/degC.
     * 
     * Keep the max current far below the maximum continuous limit (one
     * amp), say 0.5 amps.
     * 
     * TODO: choose a max voltage level experimentally.
     */
    private static final double kMaxVoltage = 12;
    private final CANSparkMax led;

    public Illuminator() {
        // note that "smart" current limiting appears to be for brushless motors only,
        // and "secondary" current limiting appears not to work (it produces an error in
        // the log, and happily outputs max current), so we don't use either one.
        led = new CANSparkMax(21, MotorType.kBrushed);
        led.setInverted(true);
    }

    /**
     * @param value in range [0,1]
     */
    public void set(final double value) {
        double clampedValue = MathUtil.clamp(value, 0, 1);
        double outputVolts = clampedValue * kMaxVoltage;
        led.setVoltage(outputVolts);
        double outputCurrent = led.getOutputCurrent();
        System.out.printf("output volts: %5.3f amps: %5.3f\n", outputVolts, outputCurrent);
    }
}
