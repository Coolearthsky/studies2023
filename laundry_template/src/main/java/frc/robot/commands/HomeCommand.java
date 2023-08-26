package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LaundryArm;

public class HomeCommand extends Command {
    private LaundryArm m_arm;
    private int i = 0;
    private int j = 0;
    private boolean isFinished = false;

    public HomeCommand(LaundryArm arm) {
        m_arm = arm;
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    public void execute() {
        if (i == 0) {
            if (!m_arm.getLowerSwitch()) {
                m_arm.setOutput(.05);
                return;
            }
            i += 1;
        }
        if (i == 1) {
            if (j < 30) {
                m_arm.setOutput(-.05);
                return;
            }
            j = 0;
            i += 1;
        }
        if (i == 2) {
            if (!m_arm.getLowerSwitch()) {
                m_arm.setOutput(.05);
                return;
            }
            m_arm.limitSet();
            i += 1;
        }
        if (i == 3) {
            if (!m_arm.getUpperSwitch()) {
                m_arm.setOutput(-.05);
                return;
            }
            i += 1;
        }
        if (i == 4) {
            if (j < 30) {
                m_arm.setOutput(.05);
                return;
            }
            j = 0;
            i += 1;
        }
        if (i == 5) {
            if (!m_arm.getUpperSwitch()) {
                m_arm.setOutput(-.05);
                return;
            }
            m_arm.zeroSet();
            m_arm.setOutput(0);
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
