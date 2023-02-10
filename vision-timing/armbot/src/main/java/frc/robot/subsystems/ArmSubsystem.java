package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class ArmSubsystem extends ProfiledPIDSubsystem {
    public static final int kMotorPort = 4;
    public static final double kP = 1;
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;
    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 10;
    public static final int[] kEncoderPorts = new int[] { 4, 5 };
    public static final int kEncoderPPR = 256;
    public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;

    public static final double kArmOffsetRads = 0.5;

    private final PWMSparkMax m_motor = new PWMSparkMax(kMotorPort);
    private final Encoder m_encoder = new Encoder(kEncoderPorts[0], kEncoderPorts[1]);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            kSVolts, kGVolts,
            kVVoltSecondPerRad, kAVoltSecondSquaredPerRad);

    /** Create a new ArmSubsystem. */
    public ArmSubsystem() {
        super(
                new ProfiledPIDController(
                        kP,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                kMaxVelocityRadPerSecond,
                                kMaxAccelerationRadPerSecSquared)),
                0);
        m_encoder.setDistancePerPulse(kEncoderDistancePerPulse);
        // Start arm at rest in neutral position
        setGoal(kArmOffsetRads);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        m_motor.setVoltage(output + feedforward);
    }

    @Override
    public double getMeasurement() {
        return m_encoder.getDistance() + kArmOffsetRads;
    }
}
