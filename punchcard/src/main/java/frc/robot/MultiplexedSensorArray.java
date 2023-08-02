package frc.robot;

/**
 * Represents the Pololu QTRX-HD-05A which is a 5-channel reflective infrared
 * photosensor, combined with the Sparkfun 16-channel multiplexer to address each output.
 * 
 * Pololu product page:
 * 
 * https://www.pololu.com/product/4405/resources
 */
public class MultiplexedSensorArray {
    private final ReflectiveSensor[] m_sensors;
    private final Mux m_mux;

    public MultiplexedSensorArray(ReflectiveSensor[] sensors, Mux mux) {
        m_sensors = sensors;
        m_mux = mux;
    }

    public boolean[] read() {
        // to read all the bits, we have to scan with the mux.
        boolean[] result = new boolean[m_sensors.length];
        for (int i = 0; i < m_sensors.length; ++i) {
            m_mux.set(i);
            try {
                Thread.sleep(0, 100);// 100ns of settling time
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            result[i] = m_sensors[i].read();
        }
        return result;
    }
}
