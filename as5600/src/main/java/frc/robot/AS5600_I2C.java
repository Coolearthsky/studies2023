package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.geometry.Rotation2d;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.function.Supplier;

/**
 * Datasheet is here:
 * 
 * https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
 */
public class AS5600_I2C implements Supplier<Rotation2d> {
    private static final byte kAddress = (byte) 0x36;
    private static final byte kAngleRaw = (byte) 0x0c;

    private final I2C m_i2c;

    public AS5600_I2C() {
        m_i2c = new I2C(I2C.Port.kMXP, kAddress);
    }

    @Override
    public Rotation2d get() {
        ByteBuffer buf = ByteBuffer.allocate(2);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(kAngleRaw, 2, buf);
        short xValue = buf.getShort();
        return Rotation2d.fromRotations((double) xValue / 4096);
    }
}
