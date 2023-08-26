package org.team100.annotations;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.junit.jupiter.api.Test;

public class AnnotationsTest {
    private static class AnnoTest {
        @Nullable
        public static String intolerant(@NotNull String foo) {
            return "foo";
        }
    }

    // annotation is required here; the dereference below will not be noticed.
    private static String bystander(@NotNull String foo) {
        return foo.toUpperCase();
    }

    private static String middleman(String foo) {
        return bystander(foo);
    }

    @Test
    void testNothing() {
        String foo = AnnoTest.intolerant(null);
        String FOO = foo.toUpperCase();
        assertEquals("FOO", FOO);
    }

    @Test
    void testBystander() {
        String foo = AnnoTest.intolerant(null);
        String FOO2 = middleman(foo);
        assertEquals("FOO2", FOO2);
    }
}
