package org.team100.frc2023.midi;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.MidiEvent;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.Sequence;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.Track;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Read a MIDI file.
 * I don't use the java sequencer, I just want the timestamped events.
 */
public class MidiReader {
    private final Track[] m_tracks;

    public MidiReader() {
        // this("src/main/deploy/scales.mid");
        this("scales.mid");
    }

    public MidiReader(String filepath) {
        m_tracks = makeTracks(filepath);
    }

    public Track[] getTracks() {
        return m_tracks;
    }

    /** Dumps all the tracks to stdout. */
    public void print() {
        for (Track track : m_tracks) {
            for (int iEvent = 0; iEvent < track.size(); ++iEvent) {
                MidiEvent event = track.get(iEvent);
                MidiMessage msg = event.getMessage();
                long tick = event.getTick();
                if (!(msg instanceof ShortMessage)) {
                    System.out.println("Other message: " + msg.getClass());
                    continue;
                }
                ShortMessage sm = (ShortMessage) msg;

                if (sm.getCommand() != ShortMessage.NOTE_ON) {
                    System.out.println("Command:" + sm.getCommand());
                    continue;
                }

                // musescore 4 doesn't do channel assignment
                // https://musescore.org/en/node/339580
                // so i'll have to have a little map
                int channel = sm.getChannel();
                int key = sm.getData1();
                int velocity = sm.getData2();
                if (velocity > 0) {
                    System.out.println(
                            "Ch " + channel + " tick " + tick + " Attack  " + " key " + key + " v " + velocity);
                } else {
                    System.out.println(
                            "Ch " + channel + " tick " + tick + " Release " + " key " + key);
                }
            }
        }
    }

    /** Returns an array of tracks, empty if something goes wrong. */
    private Track[] makeTracks(String filepath) {
        Path path = Filesystem.getDeployDirectory().toPath().resolve(filepath);
        try (InputStream is = Files.newInputStream(path)) {
            return MidiSystem.getSequence(is).getTracks();
        } catch (IOException | InvalidMidiDataException e) {
            throw new RuntimeException(e);
        }
    }
}
