package org.team100.frc2023.kinematics;

/** Represents the state of a Lynx arm. */
public class LynxArmAngles {
    // these are all native servo units
    public final double swing;
    public final double boom;
    public final double stick;
    public final double wrist;
    public final double twist;
    public final double grip;

    public LynxArmAngles(
            double swing0_1,
            double boom0_1,
            double stick0_1,
            double wrist0_1,
            double twist0_1,
            double grip0_1) {
        this.swing = swing0_1;
        this.boom = boom0_1;
        this.stick = stick0_1;
        this.wrist = wrist0_1;
        this.twist = twist0_1;
        this.grip = grip0_1;
    }

    /**
     * Swing is zero at center, positive counterclockwise.
     */
    public double swingRad() {
        return (0.5 - swing) * Math.PI;
    };

    /**
     * Boom straight up is zero, positive forward.
     */
    public double boomRad() {
        return (0.5 - boom) * 4;
    };

    /**
     * Stick is **relative to boom,** unlike the arm from 2023.
     * Zero is fully extended, positive is forward.
     */
    public double stickRad() {
        return stick * Math.PI;
    };

    /**
     * Wrist is relative to stick.
     * Zero is fully extended, positive is forward.
     */
    public double wristRad() {
        return (0.55 - wrist) * Math.PI;
    };

    /**
     * @param swingRad
     * @param boomRad
     * @param stickRad
     * @param wristRad
     * @param twist0_1 passthrough not rad
     * @param grip0_1  passthrough not rad
     */
    public static LynxArmAngles fromRad(
            double swingRad,
            double boomRad,
            double stickRad,
            double wristRad,
            double twist0_1,
            double grip0_1) {
        return new LynxArmAngles(
                0.5 - swingRad / Math.PI,
                0.5 - boomRad / 4,
                stickRad / Math.PI,
                0.55 - wristRad / Math.PI,
                twist0_1,
                grip0_1);
    }

}
