package team100.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.simulation.AnalogEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Simplest possible Arm, kinda like the one we're using. */
public class Arm extends SubsystemBase {
    // hardware
    private final CANSparkMax motor;
    private final AnalogEncoder encoder;

    // simulation
    private final SingleJointedArmSim armSim;
    private final AnalogEncoderSim encoderSim;

    // visualization
    private final Mechanism2d m_mech2d;
    private final MechanismRoot2d m_armPivot;
    private final MechanismLigament2d m_armTower;
    private final MechanismLigament2d m_arm;

    public Arm(int motorCanId, int encoderChannel) {
        // hardware
        motor = new CANSparkMax(motorCanId, MotorType.kBrushless);
        encoder = new AnalogEncoder(encoderChannel);
        encoder.setDistancePerRotation(Math.PI * 2.0);

        // simulation
        final DCMotor motorConstants = DCMotor.getNEO(1);
        final double massKg = 1;
        final double lengthM = 1;
        final double momentOfIntertiaJKg2 = SingleJointedArmSim.estimateMOI(lengthM, massKg);
        final double gearing = 30;

        LinearSystem<N2, N1, N1> plant = LinearSystemId.createSingleJointedArmSystem(
                motorConstants, momentOfIntertiaJKg2, gearing);
        armSim = new SingleJointedArmSim(
                plant,
                motorConstants,
                gearing,
                lengthM,
                -Math.PI / 2, // min angle radians
                Math.PI, // max angle radians
                massKg,
                true // simulate gravity?
        );
        encoderSim = new AnalogEncoderSim(encoder);

        // visualization
        m_mech2d = new Mechanism2d(60, // width
                60); // height
        m_armPivot = m_mech2d.getRoot("ArmPivot", 30, // x
                30);// y
        m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, // length
                -90));// angle degrees
        m_armTower.setColor(new Color8Bit(Color.kBlue));
        m_arm = m_armPivot.append(
                new MechanismLigament2d(
                        "Arm",
                        30, // length
                        Units.radiansToDegrees(armSim.getAngleRads()),
                        6, // width
                        new Color8Bit(Color.kYellow)));

        SmartDashboard.putData("arm sim", m_mech2d);
    }

    /** @param output [-1,1] */
    public void set(double output) {
        motor.set(output);
    }

    /** @return position in radians */
    public double get() {
        return encoder.getDistance();
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(motor.get() * 12); // volts
        armSim.update(0.02);
        encoderSim.setPosition(new Rotation2d(armSim.getAngleRads()));
        m_arm.setAngle(new Rotation2d(armSim.getAngleRads()));
    }

}
