package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LaundryArm extends Subsystem {
    private ProfiledPIDController m_controller;
    private CANSparkMax m_motor;
    private DigitalInput m_upperSwitch;
    private DigitalInput m_lowerSwitch;
    private double kOffset;
    private double kLimit;
    private double kGoal;
    public LaundryArm(ProfiledPIDController controller, CANSparkMax motor, DigitalInput upperSwitch,
            DigitalInput lowerSwitch) {
        m_controller = controller;
        m_motor = motor;
        m_upperSwitch = upperSwitch;
        m_lowerSwitch = lowerSwitch;

        m_motor.enableVoltageCompensation(12.0);
    }

    public void zeroSet() {
        kOffset = getAbsolute();
    }

    public void limitSet() {
        kLimit = getAbsolute();
    }

    public void setOutput(double output) {
        m_motor.set(output);
    }

    public double getAbsolute() {
        return m_motor.getEncoder().getPosition();
    }

    public double calculate(double goal) {
        return m_controller.calculate(getAbsolute(), goal);
    }

    public void setDegrees(double goal) {
        kGoal = MathUtil.clamp((goal/360)+kOffset, kOffset, kLimit);
    }

    public boolean getUpperSwitch() {
        return m_upperSwitch.get();
    }

    public boolean getLowerSwitch() {
        return m_lowerSwitch.get();
    }

    @Override
    public void periodic() {
        setOutput(calculate(kGoal));
    }
}
