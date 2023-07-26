package org.team100.frc2023.commands;

import java.util.function.Supplier;

import org.team100.frc2023.kinematics.LynxArmAngles;
import org.team100.frc2023.subsystems.Arm;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Captures goals at initialization time, moves the arm. */
public class MoveAllAxes extends Command {
    private static double kDt = 0.02;
    private final Supplier<LynxArmAngles> m_input;
    private final Arm m_arm;

    /**
     * @param hold means never complete, i.e. hold while button is held.
     */
    public MoveAllAxes(Supplier<LynxArmAngles> input, Arm output) {
        m_input = input;
        m_arm = output;
        addRequirements(m_arm);
        SmartDashboard.putData("move all axes", this);
    }

    /** Records the goals at the moment the command starts. */
    @Override
    public void initialize() {
        m_arm.setGoals(m_input.get());
    }

    @Override
    public void execute() {
       // m_arm.move(kDt);
    }

    @Override
    public boolean isFinished() {
        return m_arm.atGoal();
    }



    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("isFinished", this::isFinished, null);
    }
}