package org.team100.instrument;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.frc2023.instrument.Piano;

public class PianoTest {
    private static final double kDelta = 0.001;
    @Test
    public void testInvalidMidi() {
        Piano p = new Piano();
        assertNull(p.get(35)); // only down to 36
    }
    
    @Test
    public void testMiddleC() {
        Piano p = new Piano();
        Piano.Key c4 = p.get(60);
        assertEquals("C4", c4.name);
        assertEquals(14, c4.x, kDelta);
        assertTrue(c4.isWhite());
        Piano.Key ctr = p.get(59);
        assertEquals(0.0232, c4.x(ctr), kDelta);
        assertEquals(0, c4.y(), kDelta);
    }

    @Test
    public void testMiddleCSharp() {
        Piano p = new Piano();
        Piano.Key c4 = p.get(61);
        assertEquals("C#4", c4.name);
        assertEquals(14.366, c4.x, kDelta);
        assertFalse(c4.isWhite());
        Piano.Key ctr = p.get(59);
        assertEquals(0.0317, c4.x(ctr), kDelta);
        assertEquals(0.045, c4.y(), kDelta);

    }
}
