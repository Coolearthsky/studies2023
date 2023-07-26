package org.team100.frc2023.commands;

import java.util.List;

import org.team100.frc2023.kinematics.LynxArmAngles;
import org.team100.frc2023.subsystems.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Move according to a schedule. */
public class MoveSequence extends Command {
    public static class Event {
        public final double timeSec;
        public final LynxArmAngles angles;

        public Event(double timeSec, LynxArmAngles angles) {
            this.timeSec = timeSec;
            this.angles = angles;
        }
    }

    private final Arm m_arm;
    private final Timer m_timer;
    private final List<Event> m_events;
    private int m_nextEvent;

    public MoveSequence(Arm arm, List<Event> events) {
        m_arm = arm;
        m_timer = new Timer();
        m_events = events;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_nextEvent = 0;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double currentTimeSec = m_timer.get();
        Event event = m_events.get(m_nextEvent);
        if (event.timeSec < currentTimeSec) {
            m_arm.setGoals(event.angles);
            m_nextEvent = m_nextEvent + 1;
        }
    }

    @Override
    public boolean isFinished() {
        return m_nextEvent == m_events.size();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }
}
