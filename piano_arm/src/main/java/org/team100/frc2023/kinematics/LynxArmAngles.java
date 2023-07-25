package org.team100.frc2023.kinematics;


/** Represents the state of a Lynx arm. */
public class LynxArmAngles {
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

}
