
package org.team100.frc2024;

import org.team100.lib.barcode.Sensor;
import org.team100.lib.barcode.Mux;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
    private final DigitalOutput[] m_outputs = new DigitalOutput[] {
            new DigitalOutput(0),
            new DigitalOutput(1),
            new DigitalOutput(2),
            new DigitalOutput(3)
    };
    // TODO: wire up the mux and try it out
    private final Mux mux = new Mux(m_outputs);
    private final AnalogInput m_input = new AnalogInput(0);
    private final AnalogInput m_testInput = new AnalogInput(1);
    private final DigitalOutput m_odd = new DigitalOutput(5);
    private final DigitalOutput m_even = new DigitalOutput(6);
    private final Timer m_testTimer = new Timer();
    // TODO: tune the thresholds
    private final Sensor m_array = new Sensor(
            m_input,
            new double[] { 2.5, 3, 1.5, 2.4, 1.8 },
            mux,
            m_odd,
            m_even);

    private int id;

    public Robot() {
    }

    @Override
    public void robotInit() {
        m_testTimer.start();
        // TODO: represent failure in a better way than -1.
        id = m_array.readValue().orElse(-1);
    }

    @Override
    public void teleopPeriodic() {
        System.out.println(m_array.readValue());
        System.out.printf("0: %5.3f 1: %5.3f\n",
                    m_input.getAverageVoltage(),
                    m_testInput.getAverageVoltage());
    }

    @Override
    public void testPeriodic() {
        if (m_testTimer.get() > 0.25) {
            System.out.printf("0: %5.3f 1: %5.3f\n",
                    m_input.getAverageVoltage(),
                    m_testInput.getAverageVoltage());
            m_testTimer.restart();
        }
    }

    @Override
    public void close() {
        super.close();
        for (int i = 0; i < m_outputs.length; ++i) {
            m_outputs[i].close();
        }
    }
}
