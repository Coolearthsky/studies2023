package org.team100.kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.frc2023.kinematics.LynxArmAngles;

public class LynxArmAnglesTest {
    private static final double kDelta = 0.001;

    @Test
    public void testSwingRad() {
        assertEquals(Math.PI / 2, new LynxArmAngles(0, 0, 0, 0, 0, 0).swingRad(), kDelta);
        assertEquals(Math.PI / 4, new LynxArmAngles(0.25, 0, 0, 0, 0, 0).swingRad(), kDelta);
        assertEquals(0, new LynxArmAngles(0.5, 0, 0, 0, 0, 0).swingRad(), kDelta);
        assertEquals(-Math.PI / 4, new LynxArmAngles(0.75, 0, 0, 0, 0, 0).swingRad(), kDelta);
        assertEquals(-Math.PI / 2, new LynxArmAngles(1, 0, 0, 0, 0, 0).swingRad(), kDelta);

        assertEquals(0, LynxArmAngles.fromRad(Math.PI / 2, 0, 0, 0, 0, 0).swing, kDelta);
        assertEquals(0.25, LynxArmAngles.fromRad(Math.PI / 4, 0, 0, 0, 0, 0).swing, kDelta);
        assertEquals(0.5, LynxArmAngles.fromRad(0, 0, 0, 0, 0, 0).swing, kDelta);
        assertEquals(0.75, LynxArmAngles.fromRad(-Math.PI / 4, 0, 0, 0, 0, 0).swing, kDelta);
        assertEquals(1, LynxArmAngles.fromRad(-Math.PI / 2, 0, 0, 0, 0, 0).swing, kDelta);

    }

    @Test
    public void testBoomRad() {
        assertEquals(1, new LynxArmAngles(0, 0.25, 0, 0, 0, 0).boomRad(), kDelta);
        assertEquals(0, new LynxArmAngles(0, 0.5, 0, 0, 0, 0).boomRad(), kDelta);
        assertEquals(-1, new LynxArmAngles(0, 0.75, 0, 0, 0, 0).boomRad(), kDelta);

        assertEquals(0.75, LynxArmAngles.fromRad(0, -1, 0, 0, 0, 0).boom, kDelta);
        assertEquals(0.5, LynxArmAngles.fromRad(0, 0, 0, 0, 0, 0).boom, kDelta);
        assertEquals(0.25, LynxArmAngles.fromRad(0, 1, 0, 0, 0, 0).boom, kDelta);
    }

    @Test
    public void testStickRad() {
        assertEquals(Math.PI / 4, new LynxArmAngles(0, 0, 0.25, 0, 0, 0).stickRad(), kDelta);
        assertEquals(Math.PI / 2, new LynxArmAngles(0, 0, 0.5, 0, 0, 0).stickRad(), kDelta);
        assertEquals(Math.PI, new LynxArmAngles(0, 0, 1, 0, 0, 0).stickRad(), kDelta);

        assertEquals(0.25, LynxArmAngles.fromRad(0, 0, Math.PI / 4, 0, 0, 0).stick, kDelta);
        assertEquals(0.5, LynxArmAngles.fromRad(0, 0, Math.PI / 2, 0, 0, 0).stick, kDelta);
        assertEquals(1, LynxArmAngles.fromRad(0, 0, Math.PI, 0, 0, 0).stick, kDelta);
    }

    @Test
    public void testWristRad() {
        assertEquals(-Math.PI / 4, new LynxArmAngles(0, 0, 0, 0.8, 0, 0).wristRad(), kDelta);
        assertEquals(0, new LynxArmAngles(0, 0, 0, 0.55, 0, 0).wristRad(), kDelta);
        assertEquals(Math.PI / 4, new LynxArmAngles(0, 0, 0, 0.3, 0, 0).wristRad(), kDelta);

        assertEquals(0.8, LynxArmAngles.fromRad(0, 0, 0, -Math.PI / 4, 0, 0).wrist, kDelta);
        assertEquals(0.55, LynxArmAngles.fromRad(0, 0, 0, 0, 0, 0).wrist, kDelta);
        assertEquals(0.3, LynxArmAngles.fromRad(0, 0, 0, Math.PI / 4, 0, 0).wrist, kDelta);

    }

}
