package team100;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

/**
 * Example LED illuminator using a triple Cree XP-E2 from LED Supply, driven
 * with a REV SparkMax controller.
 * 
 * The LEDs are described here:
 * 
 * https://assets.cree-led.com/a/ds/x/XLamp-XPE2.pdf
 * 
 * For green, the max current is 1.5 A, and the forward voltage at max current
 * is about 3.2V, or 9.6V for the triple.
 * 
 * I measured the IV curve of the Cree XP-E2 green triple here:
 * https://docs.google.com/spreadsheets/d/1mFUEQ6ZgwABm4Fxg8D6743Wt6MMn2E7xbcdl7Rf4_sY/edit#gid=0
 * 
 * Note the ammeter shunt accounts for tens of millivolts in this chart,
 * and the voltage is the *requested* voltage, not the voltage across the LED.
 * 
 * These results indicate that it's not possible to exceed the maximum allowed
 * current even with the full 12 V applied, so the device should be safe from
 * overload.
 * 
 * Note that getOutputCurrent() is complete garbage; I used a real ammeter.
 */
public class Illuminator {
    /**
     * There is no current-limiting resistor in this setup, we rely on voltage to
     * choose an output level; this has the advantage of simplicity and avoiding a
     * multi-watt resistor, but it has the disadvantage that the LED output is not
     * constant with voltage, it depends on temperature in a thermal-runaway
     * fashion. See https://assets.cree-led.com/a/ds/x/XLamp-XPE2.pdf page 3:
     * temperature coefficient of green is -1.2 mV/C, red is -1.8 mV/C.
     * 
     * Say the junction is 100 C so the forward voltage droops by about 0.2 V.
     * So keep the max voltage, say, 0.5 V away from the maximum to account
     * for heating.
     * 
     * But since the maximum is higher than the supply voltage, I think 12 is fine.
     */
    private static final double kMaxVoltage = 12.0;
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
    }
}
