package org.team100.frc2023;

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
public enum Piano {
    C1(-17, true),
    D1(-16, true),
    E1(-15, true),
    F1(-14, true),
    G1(-13, true),
    A1(-12, true),
    B1(-11, true),
    C2(-10, true),
    D2(-9, true),
    E2(-8, true),
    F2(-7, true),
    G2(-6, true),
    A2(-6, true),
    B2(-5, true),
    C3(-4,true),
    D3(-3, true),
    E3(-2, true),
    F3(-1, true),
    G3(0, true);

    private static final double w = 0.08;
    private static final double b = 0.13;
    private static final double s = 0.0232; // meters
    private final double x;
    private final double y;

    private Piano(int x, boolean white) {
        this.x = x * s;
        this.y = white ? w : b;
    }

}
