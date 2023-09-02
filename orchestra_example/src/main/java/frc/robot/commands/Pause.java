package frc.robot.commands;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.Command;

public class Pause extends Command {
    private Orchestra m_orchestra;
    private boolean isFinished;
    public Pause(Orchestra orchestra) {
        m_orchestra = orchestra;
    }

    @Override
    public void initialize() {
        isFinished = false;
    }
    @Override
    public void execute() {
        if (m_orchestra.isPlaying()) {
            m_orchestra.pause();
        }
        else {
            m_orchestra.play();
        }
        isFinished = true;
    }
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
