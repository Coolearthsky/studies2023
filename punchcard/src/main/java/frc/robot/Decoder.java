package frc.robot;

/**
 * Decode the sensor readings into a number.
 */
public class Decoder {
    private final MultiplexedSensorArray m_array;

    public Decoder(MultiplexedSensorArray array) {
        m_array = array;
    }

    public int read() {
        return Util.toChannel(m_array.read());
    }

}
