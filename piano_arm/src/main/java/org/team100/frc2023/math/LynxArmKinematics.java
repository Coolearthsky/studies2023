package org.team100.frc2023.math;

import edu.wpi.first.math.geometry.Translation3d;

import org.team100.frc2023.kinematics.LynxArmAngles;

/**
 * The six-dof arm is used as a four-dof here:
 * swing, boom, stick, wrist.
 * the twist is always level, the grip is always closed.
 */
public class LynxArmKinematics {
    public final double m_boomLength;// = 20; // l1
    public final double m_stickLength;// = 20; // l2
    public final double m_wristLength;// = 5; // l3

    private final TwoDofKinematics twodof;

    public LynxArmKinematics(double boomLength, double stickLength, double wristLength) {
        m_boomLength = boomLength;
        m_stickLength = stickLength;
        m_wristLength = wristLength;
        twodof = new TwoDofKinematics(boomLength, stickLength);
    }

    /**
     * @returns relative to the arm origin, which is the intersection of the
     *          swing and boom axes.
     *          Facing the keys, +x is right, +y is forward, +z is up.
     */
    public Translation3d forward(LynxArmAngles joints) {
        // first find the 2d solution, in the 2dof axes (x up, y fwd)
        double up = m_boomLength * Math.cos(joints.boom)
                + m_stickLength * Math.cos(joints.stick + joints.boom)
                + m_wristLength * Math.cos(joints.wrist + joints.stick + joints.boom);
        double out = m_boomLength * Math.sin(joints.boom)
                + m_stickLength * Math.sin(joints.stick + joints.boom)
                + m_wristLength * Math.sin(joints.wrist + joints.stick + joints.boom);
        // then rotate it by the swing.

        return new Translation3d(
                out * Math.sin(joints.swing),
                out * Math.cos(joints.swing),
                up);
    }

    /**
     * @param position relative to the arm origin, which is the intersection of the
     *                 swing and boom axes.
     */
    public LynxArmAngles inverse(Translation3d position) {
        return null;
    }

}