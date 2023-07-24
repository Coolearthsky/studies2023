
package frc.robot;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.MidiEvent;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.MidiUnavailableException;
import javax.sound.midi.Receiver;
import javax.sound.midi.Sequence;
import javax.sound.midi.Sequencer;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.Track;
import javax.sound.midi.Transmitter;
import javax.sound.midi.Sequencer.SyncMode;

public final class Main {

    public static final String[] NOTE_NAMES = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };

    public static void main(String... args) throws MidiUnavailableException, IOException, InvalidMidiDataException {

        Sequencer sequencer = MidiSystem.getSequencer();
        System.out.println(sequencer.getClass().getName());
        InputStream is = new BufferedInputStream(new FileInputStream(new File("src/main/deploy/Autumn_Leaves.mid")));
        sequencer.setSequence(is);
        Sequence sequence = sequencer.getSequence();
        Track[] tracks = sequence.getTracks();
        for (Track track : tracks) {
            for (int iEvent = 0; iEvent < track.size(); ++iEvent) {
                MidiEvent event = track.get(iEvent);
                MidiMessage msg = event.getMessage();
                long tick = event.getTick();
                System.out.println("msg: " + msg.getClass());
                if (!(msg instanceof ShortMessage)) {
                    System.out.println("Other message: " + msg.getClass());
                    continue;
                }
                ShortMessage sm = (ShortMessage) msg;

                if (sm.getCommand() != ShortMessage.NOTE_ON) {
                    System.out.println("Command:" + sm.getCommand());
                    continue;
                }

                // there are two channels in this particular recording.
                // musescore 4 doesn't do channel assignment
                // https://musescore.org/en/node/339580
                // so i'll have to have a little map
                int channel = sm.getChannel();

                int key = sm.getData1();
                int octave = (key / 12) - 1;
                int note = key % 12;
                String noteName = NOTE_NAMES[note];
                int velocity = sm.getData2();
                if (velocity > 0) {
                    System.out.println("Channel: " + channel + " tick " + tick + " Attack,  " + noteName + octave
                            + " key=" + key + " velocity: " + velocity);
                } else {
                    System.out.println(
                            "Channel: " + channel + " tick " + tick + " Release, " + noteName + octave + " key=" + key);
                }

            }
        }
        Transmitter trans = sequencer.getTransmitter();
        Receiver rcvr = new TextReceiver();
        trans.setReceiver(rcvr);
        sequencer.open();

        sequencer.start();
    }
}
