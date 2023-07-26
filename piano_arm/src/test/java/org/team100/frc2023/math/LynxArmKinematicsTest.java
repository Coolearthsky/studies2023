package org.team100.frc2023.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.frc2023.kinematics.LynxArmAngles;

import edu.wpi.first.math.geometry.Translation3d;

public class LynxArmKinematicsTest {
    private static final double kDelta = 0.001;
    @Test
    void testf1() {
        LynxArmKinematics k = new LynxArmKinematics(1,1,1);
        LynxArmAngles a = new LynxArmAngles(0,0,0,0,0,0);
        Translation3d t = k.forward(a);
        assertEquals(0, t.getX(), kDelta);
        assertEquals(0, t.getY(), kDelta);
        assertEquals(3, t.getZ(), kDelta);
    }
    
}
