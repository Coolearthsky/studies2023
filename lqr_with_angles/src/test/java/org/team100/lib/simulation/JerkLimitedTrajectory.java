package org.team100.lib.simulation;

import org.junit.jupiter.api.Test;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;

/**
 * WPI profiles have infinite jerk which makes the feedback controller ring.
 * 
 * This uses the roadrunner jerk-limited profiles and doesn't ring.
 * 
 * Results here:
 * 
 * https://docs.google.com/spreadsheets/d/1miehTmvbdRFs49wy2x8u_3SqVKrNw-oJB5Dv5-uQYo0
 */
public class JerkLimitedTrajectory extends Scenario {
    private static final double maxVel = 0.5;
    private static final double maxAccel = 1.5;
    private static final double maxJerk = 1;

    private final MotionState start;
    private final MotionState goal;
    private final MotionProfile profile;

    public JerkLimitedTrajectory() {
        start = new MotionState(0, 0, 0);
        goal = new MotionState(1, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                goal,
                maxVel,
                maxAccel,
                maxJerk);
    }

    @Override
    String label() {
        return "TRAJECTORY";
    }

    @Override
    double position(double timeSec) {
        MotionState s = profile.get(timeSec);
        return s.getX();
    }

    @Override
    double velocity(double timeSec) {
        MotionState s = profile.get(timeSec);
        return s.getV();
    }

    @Override
    double acceleration(double timeSec) {
        MotionState s = profile.get(timeSec);
        return s.getA();
    }

    @Test
    public void test() {
        Loop loop = new Loop(this);
        loop.run();
    }
}
