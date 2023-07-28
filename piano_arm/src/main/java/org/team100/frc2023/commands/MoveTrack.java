package org.team100.frc2023.commands;

import javax.sound.midi.MetaMessage;
import javax.sound.midi.MidiEvent;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.Track;

import org.team100.frc2023.instrument.Piano;
import org.team100.frc2023.kinematics.LynxArmAngles;
import org.team100.frc2023.math.LynxArmKinematics;
import org.team100.frc2023.subsystems.Arm;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Move according to a schedule. */
/** Play a midi track. */
public class MoveTrack extends Command {
    public static class Event {
        public final double timeSec;
        public final LynxArmAngles angles;

        public Event(double timeSec, LynxArmAngles angles) {
            this.timeSec = timeSec;
            this.angles = angles;
        }
    }

    private final Arm m_arm;

    private final Track m_track;
    private final Piano m_piano;
    private final Piano.Key m_center;
    private final LynxArmKinematics m_kinematics;
    private final Timer m_timer;
    private int m_nextEvent;

    public MoveTrack(Arm arm, Track track, Piano piano, Piano.Key center, LynxArmKinematics kinematics) {
        m_arm = arm;

        m_track = track;
        m_piano = piano;
        m_center = center;
        m_kinematics = kinematics;
        m_timer = new Timer();
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

        if (m_track.size() == 0) {
            System.out.println("track is empty");
            return;
        }
        LynxArmAngles angles = getNote();

        if (angles == null) {
            // System.out.println("null angles");
            return;
        }

        m_arm.setGoals(angles);

    }

    /** go through events to find the next note_on message in the past */
    private LynxArmAngles getNote() {
        while (m_nextEvent < m_track.size()) {
            MidiEvent event = m_track.get(m_nextEvent);
            MidiMessage msg = event.getMessage();
            if (!(msg instanceof ShortMessage)) {
                System.out.println("Not ShortMessage: " + msg.getClass().getName());
                if (msg instanceof MetaMessage) {
                    MetaMessage meta = (MetaMessage) msg;
                    System.out.println("Meta message type " + meta.getType());
                }
                m_nextEvent += 1;
                continue;
            }

            ShortMessage sm = (ShortMessage) msg;
            if (sm.getCommand() != ShortMessage.NOTE_ON) {
                if (sm.getCommand() == ShortMessage.CONTROL_CHANGE) {
                    System.out.printf("control change %d %d\n", sm.getData1(), sm.getData2());
                } else {
                    System.out.println("Not NOTE_ON: " + sm.getCommand());
                }
                m_nextEvent += 1;
                continue;
            }

            double tSec = (double) event.getTick() / 1000;
            double currentTimeSec = m_timer.get();
           // System.out.printf("%5.3f tick %5.3f\n", currentTimeSec, tSec);

            int key = sm.getData1();
            int velocity = sm.getData2();

            if (velocity > 0) {
                if (tSec < currentTimeSec) {
                   // System.out.printf("%5.3f skip %d\n", currentTimeSec, key);
                    m_nextEvent += 1;
                    return null;
                }
                if (tSec - 0.25 > currentTimeSec) {
                   // System.out.printf("%5.3f wait for %d\n", currentTimeSec, key);
                    // not time yet
                    return null;
                }
                if (tSec - 0.2 > currentTimeSec) {
                    System.out.printf("%5.3f move to %d\n", currentTimeSec, key);
                    return up(key);
                }
                // play on time
                System.out.printf("%5.3f attack %d\n", currentTimeSec, key);
                m_nextEvent += 1;
                return down(key);
            } else {
                if (tSec - 0.3 > currentTimeSec) {
                    // not time yet
                    return null;
                }
                System.out.printf("%5.3f release %d\n", currentTimeSec, key);
                m_nextEvent += 1;
                return up(key);
            }
        }
        return null;
    }

    private LynxArmAngles up(int key) {
        Piano.Key pianoKey = m_piano.get(key);
        double x = pianoKey.x(m_center);
        double y = pianoKey.y() + 0.15;
        double z = 0.08;
        double wristDown = -Math.PI / 8;
        return m_kinematics.inverse(new Translation3d(x, y, z), 0, 0.5, 0.9).down(wristDown);
    }

    private LynxArmAngles down(int key) {
        Piano.Key pianoKey = m_piano.get(key);
        double x = pianoKey.x(m_center);
        double y = pianoKey.y() + 0.15;
        double z = 0.06;
        double wristDown = Math.PI / 12;
        return m_kinematics.inverse(new Translation3d(x, y, z), 0, 0.5, 0.9).down(wristDown);
    }

    @Override
    public boolean isFinished() {
        return m_nextEvent == m_track.size();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }
}
