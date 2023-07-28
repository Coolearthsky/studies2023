package org.team100.frc2023.util;

import java.util.ArrayList;
import java.util.List;

import javax.sound.midi.MidiEvent;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.Track;

import org.team100.frc2023.commands.MoveSequence;
import org.team100.frc2023.instrument.Piano;
import org.team100.frc2023.kinematics.LynxArmAngles;
import org.team100.frc2023.math.LynxArmKinematics;

import edu.wpi.first.math.geometry.Translation3d;

/** Translate MIDI tracks to MoveSequence events. */
public class TracksToEvents {

    private final Piano m_piano;
    private final Piano.Key m_center;
    private final LynxArmKinematics m_kinematics;

    public TracksToEvents(Piano piano, Piano.Key center, LynxArmKinematics kinematics) {
        m_piano = piano;
        m_center = center;
        m_kinematics = kinematics;
    }


    public List<MoveSequence.Event> toEvents(Track[] tracks) {
        List<MoveSequence.Event> events = new ArrayList<>();
        for (Track track : tracks) {
            for (int iEvent = 0; iEvent < track.size(); ++iEvent) {
                MidiEvent event = track.get(iEvent);
                MidiMessage msg = event.getMessage();

                if (!(msg instanceof ShortMessage))
                    continue;

                ShortMessage sm = (ShortMessage) msg;
                if (sm.getCommand() != ShortMessage.NOTE_ON)
                    continue;

                int channel = sm.getChannel();
                // for now, just use channel zero.
                if (channel != 0)
                    continue;

                // a "tick" is a millisecond, i think.
                long tick = event.getTick();
                int key = sm.getData1();

                Piano.Key pianoKey = m_piano.get(key);
                double x = pianoKey.x(m_center);
                double y = pianoKey.y() + 0.15;
                double z = 0.08;
                double wristDown = Math.PI / 8;
                double tSec = tick / 1000;

                LynxArmAngles angle = m_kinematics.inverse(new Translation3d(x, y, z), 0, 0.5, 0.9);
                int velocity = sm.getData2();
                if (velocity > 0) {
                    // play the note
                    // arrive in advance of playing the note
                    events.add(new MoveSequence.Event(tSec - 0.125, angle));
                    // play the note roughly on time
                    events.add(new MoveSequence.Event(tSec, angle.down(wristDown)));
                } else {
                    // release the note a little early
                    events.add(new MoveSequence.Event(tSec - 0.25, angle));
                }
            }
        }
        return events;
    }
}
