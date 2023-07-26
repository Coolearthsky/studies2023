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

    public LynxArmAngles(double swing, double boom, double stick, double wrist, double twist, double grip) {
        this.swing = swing;
        this.boom = boom;
        this.stick = stick;
        this.wrist = wrist;
        this.twist = twist;
        this.grip = grip;
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
     * @param twist passthrough not rad
     * @param grip  passthrough not rad
     */
    public static LynxArmAngles fromRad(
            double swingRad,
            double boomRad,
            double stickRad,
            double wristRad,
            double twist,
            double grip) {
        return new LynxArmAngles(
                0.5 - swingRad / Math.PI,
                0.5 - boomRad / 4,
                stickRad / Math.PI,
                0.55 - wristRad / Math.PI, twist, grip);
    }

}
