package frc.robot;

import javax.sound.midi.MidiMessage;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;

public class TextReceiver implements Receiver {

    // public static final int NOTE_ON = 0x90;
    // public static final int NOTE_OFF = 0x80;
    public static final String[] NOTE_NAMES = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };

    public TextReceiver() {

    }

    @Override
    public void send(MidiMessage message, long timeStamp) {
        if (!(message instanceof ShortMessage)) {
            System.out.println("Other message: " + message.getClass());
            return;
        }

        ShortMessage sm = (ShortMessage) message;
        if (sm.getCommand() != ShortMessage.NOTE_ON) {
            System.out.println("Command:" + sm.getCommand());
            return;
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
            System.out.println("Channel: " + channel + " time " + timeStamp + " Attack,  " + noteName + octave + " key=" + key + " velocity: " + velocity);
        } else {
            System.out.println("Channel: " + channel + " time " + timeStamp + " Release, " + noteName + octave + " key=" + key);
        }

    }

    @Override
    public void close() {

    }
}