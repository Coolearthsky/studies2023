package org.team100.frc2023.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.frc2023.kinematics.LynxArmAngles;

import edu.wpi.first.math.geometry.Translation3d;

public class LynxArmKinematicsTest {
    private static final double kDelta = 0.001;

    // All three segments pointing up
    @Test
    void testf1() {
        LynxArmKinematics k = new LynxArmKinematics(1,1,1);
        LynxArmAngles a = new LynxArmAngles(0,0,0,0,0,0);
        Translation3d t = k.forward(a);
        assertEquals(0, t.getX(), kDelta);
        assertEquals(0, t.getY(), kDelta);
        assertEquals(3, t.getZ(), kDelta);
    }
    @Test
    void testi1() {
        LynxArmKinematics k = new LynxArmKinematics(1,1,1);
        Translation3d t = new Translation3d(0,0,3);
        LynxArmAngles a = k.inverse(t, Math.PI/2);
        assertEquals(0, a.swing, kDelta);
        assertEquals(0, a.boom, kDelta);
        assertEquals(0, a.stick, kDelta);
        assertEquals(0, a.wrist, kDelta);
    }


    // bending forward at the elbow
    @Test
    void testf2() {
        LynxArmKinematics k = new LynxArmKinematics(1,1,1);
        LynxArmAngles a = new LynxArmAngles(0,0,Math.PI/2,0,0,0);
        Translation3d t = k.forward(a);
        assertEquals(0, t.getX(), kDelta);
        assertEquals(2, t.getY(), kDelta);
        assertEquals(1, t.getZ(), kDelta);
    }

    @Test
    void testi2() {
        LynxArmKinematics k = new LynxArmKinematics(1,1,1);
        Translation3d t = new Translation3d(0,2,1);
        LynxArmAngles a = k.inverse(t, 0);
        assertEquals(0, a.swing, kDelta);
        assertEquals(0, a.boom, kDelta);
        assertEquals(Math.PI/2, a.stick, kDelta);
        assertEquals(0, a.wrist, kDelta);
    }

    // wrist on the floor, wrist ahead
    @Test
    void testf3() {
        LynxArmKinematics k = new LynxArmKinematics(1,1,1);
        LynxArmAngles a = new LynxArmAngles(0,Math.PI/6,2*Math.PI/3,-Math.PI/3,0,0);
        assertEquals(0.5, k.boomOut(a),kDelta);
        assertEquals(0.5, k.stickOut(a),kDelta);
        assertEquals(1.0, k.wristOut(a),kDelta);
        Translation3d t = k.forward(a);
        assertEquals(0, t.getX(), kDelta);
        assertEquals(2, t.getY(), kDelta);
        assertEquals(0, t.getZ(), kDelta);
    }

    @Test
    void testi3() {
        LynxArmKinematics k = new LynxArmKinematics(1,1,1);
        Translation3d t = new Translation3d(0,2,0);
        LynxArmAngles a = k.inverse(t, 0);
        assertEquals(0, a.swing, kDelta);
        assertEquals(Math.PI/6, a.boom, kDelta);
        assertEquals(2*Math.PI/3, a.stick, kDelta);
        assertEquals(-Math.PI/3, a.wrist, kDelta);
    }

    // wrist on the floor, ahead, swung counterclockwise 30 deg.
    @Test
    void testf4() {
        LynxArmKinematics k = new LynxArmKinematics(1,1,1);
        LynxArmAngles a = new LynxArmAngles(Math.PI/6,Math.PI/6,2*Math.PI/3,-Math.PI/3,0,0);
        Translation3d t = k.forward(a);
        assertEquals(-1, t.getX(), kDelta);
        assertEquals(Math.sqrt(3), t.getY(), kDelta);
        assertEquals(0, t.getZ(), kDelta);
    }

    @Test
    void testi4() {
        LynxArmKinematics k = new LynxArmKinematics(1,1,1);
        Translation3d t = new Translation3d(-1,Math.sqrt(3),0);
        LynxArmAngles a = k.inverse(t, 0);
        assertEquals(Math.PI/6, a.swing, kDelta);
        assertEquals(Math.PI/6, a.boom, kDelta);
        assertEquals(2*Math.PI/3, a.stick, kDelta);
        assertEquals(-Math.PI/3, a.wrist, kDelta);
    }
    
}
