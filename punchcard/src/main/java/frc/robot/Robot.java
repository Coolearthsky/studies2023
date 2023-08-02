
package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
    private static final int kChannels = 5;
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
    private final Timer m_testTimer = new Timer();

    public Robot() {
        m_testTimer.start();
    }

    @Override
    public void robotPeriodic() {
        // try {
        for (int i = 0; i < kChannels; ++i) {
            // mux.set(i);
            // Thread.sleep(0, 100);// 100ns of settling time
            // final int v = m_input.getValue();
            // System.out.println(v);
        }
        // } catch (final InterruptedException e) {
        // e.printStackTrace();
        // }
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
