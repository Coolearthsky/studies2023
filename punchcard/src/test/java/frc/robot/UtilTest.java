package frc.robot;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class UtilTest {
    @Test
    void testToArray() {
        assertArrayEquals(new boolean[] { false, false, false, false }, Util.toArray(0, 4));
        assertArrayEquals(new boolean[] { true, false, false, false }, Util.toArray(1, 4));
        assertArrayEquals(new boolean[] { false, true, false, false }, Util.toArray(2, 4));
        assertArrayEquals(new boolean[] { true, true, false, false }, Util.toArray(3, 4));
        assertArrayEquals(new boolean[] { false, false, true, false }, Util.toArray(4, 4));
        assertArrayEquals(new boolean[] { true, false, true, false }, Util.toArray(5, 4));
        assertArrayEquals(new boolean[] { false, true, true, false }, Util.toArray(6, 4));
        assertArrayEquals(new boolean[] { true, true, true, false }, Util.toArray(7, 4));
        assertArrayEquals(new boolean[] { false, false, false, true }, Util.toArray(8, 4));
        assertArrayEquals(new boolean[] { true, false, false, true }, Util.toArray(9, 4));
        assertArrayEquals(new boolean[] { false, true, false, true }, Util.toArray(10, 4));
        assertArrayEquals(new boolean[] { true, true, false, true }, Util.toArray(11, 4));
        assertArrayEquals(new boolean[] { false, false, true, true }, Util.toArray(12, 4));
        assertArrayEquals(new boolean[] { true, false, true, true }, Util.toArray(13, 4));
        assertArrayEquals(new boolean[] { false, true, true, true }, Util.toArray(14, 4));
        assertArrayEquals(new boolean[] { true, true, true, true }, Util.toArray(15, 4));
    }

    @Test
    void testToChannel() {
        assertEquals(0, Util.toChannel(new boolean[] { false, false, false, false }));
        assertEquals(1, Util.toChannel(new boolean[] { true, false, false, false }));
        assertEquals(2, Util.toChannel(new boolean[] { false, true, false, false }));
        assertEquals(3, Util.toChannel(new boolean[] { true, true, false, false }));
        assertEquals(4, Util.toChannel(new boolean[] { false, false, true, false }));
        assertEquals(5, Util.toChannel(new boolean[] { true, false, true, false }));
        assertEquals(6, Util.toChannel(new boolean[] { false, true, true, false }));
        assertEquals(7, Util.toChannel(new boolean[] { true, true, true, false }));
        assertEquals(8, Util.toChannel(new boolean[] { false, false, false, true }));
        assertEquals(9, Util.toChannel(new boolean[] { true, false, false, true }));
        assertEquals(10, Util.toChannel(new boolean[] { false, true, false, true }));
        assertEquals(11, Util.toChannel(new boolean[] { true, true, false, true }));
        assertEquals(12, Util.toChannel(new boolean[] { false, false, true, true }));
        assertEquals(13, Util.toChannel(new boolean[] { true, false, true, true }));
        assertEquals(14, Util.toChannel(new boolean[] { false, true, true, true }));
        assertEquals(15, Util.toChannel(new boolean[] { true, true, true, true }));

    }
}
