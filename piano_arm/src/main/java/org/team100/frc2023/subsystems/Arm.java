package org.team100.frc2023.subsystems;

import org.team100.frc2023.kinematics.LynxArmAngles;
import org.team100.frc2023.motor.ProfiledServo;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Represents the arm.
 * Coordinates motion of multiple profiled servos.
 * This is "live" all the time; it's never not going towards a goal.
 */
public class Arm extends Subsystem {
    private final static LynxArmAngles initial = new LynxArmAngles(0.25, 0.61, 0.795, 0.73, 0.5, 0.9);
    // public enum Axis {
    // Swing, Boom, Stick, Wrist, Twist, Grip
    // }

    // MXP PWM output
    // public final ProfiledServo swing = new ProfiledServo("Swing", 10);
    // public final ProfiledServo boom = new ProfiledServo("Boom", 11);
    // public final ProfiledServo stick = new ProfiledServo("Stick", 12);
    // public final ProfiledServo wrist = new ProfiledServo("Wrist", 13);
    // public final ProfiledServo twist = new ProfiledServo("Twist", 14);
    // public final ProfiledServo grip = new ProfiledServo("Grip", 15);

    // on-board PWM output
    public final ProfiledServo swing = new ProfiledServo("Swing", 0, initial.swing);
    public final ProfiledServo boom = new ProfiledServo("Boom", 1, initial.boom);
    public final ProfiledServo stick = new ProfiledServo("Stick", 2, initial.stick);
    public final ProfiledServo wrist = new ProfiledServo("Wrist", 3, initial.wrist);
    public final ProfiledServo twist = new ProfiledServo("Twist", 4, initial.twist);
    public final ProfiledServo grip = new ProfiledServo("Grip", 5, initial.grip);

    public Arm() {
        setRawGoals(initial);
    }

    /*
     * Applies the specified goals, adjusting velocity/acceleration so that all the
     * axes will complete at the same time.
     * 
     * TODO: add boundary enforcement (e.g. raise boom to avoid hitting the table).
     */
    public void setGoals(LynxArmAngles goals) {
        double slowest_eta = slowestEta(goals);
        setGoal(swing, goals.swing, slowest_eta);
        setGoal(boom, goals.boom, slowest_eta);
        setGoal(stick, goals.stick, slowest_eta);
        setGoal(wrist, goals.wrist, slowest_eta);
        setGoal(twist, goals.twist, slowest_eta);
        setGoal(grip, goals.grip, slowest_eta);
    }

    private void setRawGoals(LynxArmAngles goals) {
        swing.setGoal(goals.swing, 1);
        boom.setGoal(goals.boom, 1);
        stick.setGoal(goals.stick, 1);
        wrist.setGoal(goals.wrist, 1);
        twist.setGoal(goals.twist, 1);
        grip.setGoal(goals.grip, 1);
    }

    @Override
    public void periodic() {
        move(0.02);
    }

    /** Moves the servos. Should be called often. */
    public void move(double dt) {
        swing.move(dt);
        boom.move(dt);
        stick.move(dt);
        wrist.move(dt);
        twist.move(dt);
        grip.move(dt);
    }

    /** The arm is at its goal if all the servos are at their goals. */
    public boolean atGoal() {
        if (!swing.atGoal())
            return false;
        if (!boom.atGoal())
            return false;
        if (!stick.atGoal())
            return false;
        if (!wrist.atGoal())
            return false;
        if (!twist.atGoal())
            return false;
        if (!grip.atGoal())
            return false;
        return true;
    }

    // public double getPosition(Axis axis) {
    // return m_servos.get(axis).getPosition();
    // }

    /////////////////////////////////////////////

    private void setGoal(ProfiledServo servo, double goal, double slowest_eta) {
        double eta = servo.eta(goal);
        servo.setGoal(goal, eta / slowest_eta);
    }

    /** Finds the slowest axis; use this to slow down the other axes. */
    private double slowestEta(LynxArmAngles goals) {
        double slowest_eta = 0;
        slowest_eta = Math.max(slowest_eta, swing.eta(goals.swing));
        slowest_eta = Math.max(slowest_eta, boom.eta(goals.boom));
        slowest_eta = Math.max(slowest_eta, stick.eta(goals.stick));
        slowest_eta = Math.max(slowest_eta, wrist.eta(goals.wrist));
        slowest_eta = Math.max(slowest_eta, twist.eta(goals.twist));
        slowest_eta = Math.max(slowest_eta, grip.eta(goals.grip));
        return slowest_eta;
    }
}