package org.team100.frc2023.instrument;

import java.util.Map;

/**
 * My piano keys are 23.2mm wide.
 * The white key target is 120mm from the swing pivot.
 * The black key target is 170mm from the swing pivot.
 * the black keys are not quite centered on the white-key boundaries. Come back
 * to that.
 * the swing pivot will be set in front of the "G" key.
 * So this is the array of notes in cartesian coordinates measured from the
 * swing pivot.
 * x is along the keyboard, positive right.
 * all in meters.
 * 
 */
public class Piano {

    public class Key {
        /** the "iso" name, only useful for debugging */
        public final String name;
        /** measured in key widths, relative to C2. */
        public final double x;

        public boolean isWhite() {
            if ((x % 1) < 1e-6)
                return true;
            return false;
        }

        /** x (right positive) offset (m) of the key, relative to the given midi */
        public double x(Key ctr) {
            if (!ctr.isWhite())
                throw new IllegalArgumentException("offset must be a white key");
            return (x - ctr.x) * keyWidthM;
        }

        /** y (ahead positive) offset (m) of the key */
        public double y() {
            if (isWhite())
                return 0;
            return blackKeyOffsetM;
        }

        private Key(String name, double x) {
            this.name = name;
            this.x = x;
        }

    }

    /** key = MIDI number */
    private final Map<Integer, Key> keys = Map.ofEntries(
            key(96, "C7", 35),
            key(95, "B6", 34),
            key(94, "A#6", 33.634),
            key(93, "A6", 33),
            key(92, "G#6", 32.5),
            key(91, "G6", 32),
            key(90, "F#6", 31.366),
            key(89, "F6", 31),
            key(88, "E6", 30),
            key(87, "D#6", 29.634),
            key(86, "D6", 29),
            key(85, "C#6", 28.366),
            key(84, "C6", 28),
            key(83, "B5", 27),
            key(82, "A#5", 26.634),
            key(81, "A5", 26),
            key(80, "G#5", 25.5),
            key(79, "G5", 25),
            key(78, "F#5", 24.366),
            key(77, "F5", 24),
            key(76, "E5", 23),
            key(75, "D#5", 22.634),
            key(74, "D5", 22),
            key(73, "C#5", 21.366),
            key(72, "C5", 21),
            key(71, "B4", 20),
            key(70, "A#4", 19.634),
            key(69, "A4", 19),
            key(68, "G#4", 18.5),
            key(67, "G4", 18),
            key(66, "F#4", 17.366),
            key(65, "F4", 17),
            key(64, "E4", 16),
            key(63, "D#4", 15.634),
            key(62, "D4", 15),
            key(61, "C#4", 14.366),
            key(60, "C4", 14),
            key(59, "B3", 13),
            key(58, "A#3", 12.634),
            key(57, "A3", 12),
            key(56, "G#3", 11.5),
            key(55, "G3", 11),
            key(54, "F#3", 10.366),
            key(53, "F3", 10),
            key(52, "E3", 9),
            key(51, "D#3", 1),
            key(50, "D3", 8),
            key(49, "C#3", 7.366),
            key(48, "C3", 7),
            key(47, "B2", 6),
            key(46, "A#2", 5.634),
            key(45, "A2", 5),
            key(44, "G#2", 4.5),
            key(43, "G2", 4),
            key(42, "F#2", 3.366),
            key(41, "F2", 3),
            key(40, "E2", 2),
            key(39, "D#2", 1.634),
            key(38, "D2", 1),
            key(37, "C#2", 0.366),
            key(36, "C2", 0));

    private Map.Entry<Integer, Key> key(int midi, String name, double x) {
        return Map.entry(midi, new Key(name, x));
    }

    private final double keyWidthM;
    private final double blackKeyOffsetM;

    /**
     * Represents the keyboard i happen to have, which goes from C2 to C7 inclusive,
     * which is
     * also the common 61-key "organ" size.
     */
    public Piano(double keyWidthM, double blackKeyOffsetM) {
        this.keyWidthM = keyWidthM;
        this.blackKeyOffsetM = blackKeyOffsetM;
    }

    public Piano() {
        this(0.0232, 0.045);
    }

    public Key get(int midi) {
        return keys.get(midi);
    }

}
