package frc.robot;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;

public class MuxTest {
    /**
     * Verify the truth table.
     * https://www.sparkfun.com/datasheets/IC/cd74hc4067.pdf
     */
    @Test
    void testTruthTable() {
        // remember that the zeroth element comes first so the "place value" in this
        // list is "backwards" (just like the truth table actually)
        assertArrayEquals(new int[] { 0, 0, 0, 0 }, Mux.truth(0));
        assertArrayEquals(new int[] { 1, 0, 0, 0 }, Mux.truth(1));
        assertArrayEquals(new int[] { 0, 1, 0, 0 }, Mux.truth(2));
        assertArrayEquals(new int[] { 1, 1, 0, 0 }, Mux.truth(3));
        assertArrayEquals(new int[] { 0, 0, 1, 0 }, Mux.truth(4));
        assertArrayEquals(new int[] { 1, 0, 1, 0 }, Mux.truth(5));
        assertArrayEquals(new int[] { 0, 1, 1, 0 }, Mux.truth(6));
        assertArrayEquals(new int[] { 1, 1, 1, 0 }, Mux.truth(7));
        assertArrayEquals(new int[] { 0, 0, 0, 1 }, Mux.truth(8));
        assertArrayEquals(new int[] { 1, 0, 0, 1 }, Mux.truth(9));
        assertArrayEquals(new int[] { 0, 1, 0, 1 }, Mux.truth(10));
        assertArrayEquals(new int[] { 1, 1, 0, 1 }, Mux.truth(11));
        assertArrayEquals(new int[] { 0, 0, 1, 1 }, Mux.truth(12));
        assertArrayEquals(new int[] { 1, 0, 1, 1 }, Mux.truth(13));
        assertArrayEquals(new int[] { 0, 1, 1, 1 }, Mux.truth(14));
        assertArrayEquals(new int[] { 1, 1, 1, 1 }, Mux.truth(15));
    }
}
