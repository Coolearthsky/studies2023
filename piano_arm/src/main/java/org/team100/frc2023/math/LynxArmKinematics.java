package org.team100.frc2023.math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import org.team100.frc2023.kinematics.LynxArmAngles;
import org.team100.frc2023.math.TwoDofKinematics.ArmAngles;

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
        // this is what 2dof calls "x"
        double up = m_boomLength * Math.cos(joints.boom)
                + m_stickLength * Math.cos(joints.stick + joints.boom)
                + m_wristLength * Math.cos(joints.wrist + joints.stick + joints.boom);
        // this is what 2dof calls "y"
        double out = boomOut(joints) + stickOut(joints) + wristOut(joints);
        // then rotate it by the swing.
        return new Translation3d(
                -1 * out * Math.sin(joints.swing),
                out * Math.cos(joints.swing),
                up);
    }

    double boomOut(LynxArmAngles joints) {
        return m_boomLength * Math.sin(joints.boom);
    }

    double stickOut(LynxArmAngles joints) {
        return m_stickLength * Math.sin(joints.stick + joints.boom);
    }

    double wristOut(LynxArmAngles joints) {
        return m_wristLength * Math.sin(joints.wrist + joints.stick + joints.boom);
    }

    /**
     * since 3dof in 2d is underconstrained, specify
     * the wrist angle here to get a single solution
     * 
     * @param position              relative to the arm origin, which is the
     *                              intersection of the
     *                              swing and boom axes.
     * @param wristAngleAbsoluteRad relative to horizontal, positive up, which is
     *                              the reverse of the armangles direction.
     */
    public LynxArmAngles inverse(Translation3d position, double wristAngleAbsoluteRad) {

        double swing = -1.0 * safeAtan(position.getX(), position.getY());

        double up = position.getZ();
        double out = Math.hypot(position.getX(), position.getY());

        out = out - Math.cos(wristAngleAbsoluteRad);
        up = up - Math.sin(wristAngleAbsoluteRad);

        ArmAngles a = twodof.inverse(new Translation2d(up, out));

        double wristAngleRelative = Math.PI / 2 - wristAngleAbsoluteRad - a.th1 - a.th2;
        return new LynxArmAngles(swing, a.th1, a.th2, wristAngleRelative, 0, 0);
    }

    /** If both x and y are zero, return zero. 
     * @param x along the keyboard, right positive
     * @param y towards the keyboard
    */
    private double safeAtan(double x, double y) {
        if (Math.abs(x) < 1e-3 && Math.abs(y) < 1e-3) {
            return 0;
        }
        //// NOTE switching x and y here
        return Math.atan2(x, y);
    }

}