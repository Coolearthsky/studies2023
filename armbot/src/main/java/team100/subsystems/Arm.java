package team100.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Simplest possible Arm, kinda like the one we're using. */
public class Arm extends SubsystemBase {
    private final CANSparkMax m_motor;
    private final AnalogEncoder m_encoder;

    public Arm(int motorCanId, int encoderChannel) {
        m_motor = new CANSparkMax(motorCanId, MotorType.kBrushless);
        m_encoder = new AnalogEncoder(encoderChannel);
    }

    /** @param output [-1,1] */
    public void set(double output) {
        m_motor.set(output);
    }

    public double get() {
        return m_encoder.get();
    }
}
