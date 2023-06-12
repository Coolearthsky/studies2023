package org.team100.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionSegment;
import com.acmerobotics.roadrunner.profile.MotionState;

/**
 * Exercises the profile generator in Roadrunner.
 * 
 * Note the Roadrunner library is in Kotlin.
 *
 * Results from this test are available here:
 *
 * https://docs.google.com/spreadsheets/d/1amz21b9WxrN5SeUJqb0lktHwPy58V_cjU9X08Xd59qk
 */
public class RoadrunnerTest {
    /**
     * this is from the example
     * https://rr.brott.dev/docs/v0-5/tour/motion-profiling/
     */
    //@Test
    public void testSimple() {
        System.out.println("SIMPLE");
        MotionState start = new MotionState(0, 0, 0); // at origin at rest
        MotionState goal = new MotionState(6, 0, 0); // at 60 at rest.
        int maxVel = 3;
        int maxAccel = 6;
        int maxJerk = 12;
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                goal,
                maxVel,
                maxAccel,
                maxJerk);
        List<MotionSegment> segments = profile.getSegments();
        assertEquals(7, segments.size());

        System.out.printf("time, jerk, accel, vel, pos\n");
        for (double t = 0; t < profile.duration(); t += 0.1) {
            MotionState s = profile.get(t);
            double jerk = s.getJ();
            double accel = s.getA();
            double vel = s.getV();
            double pos = s.getX();
            System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n", t, jerk, accel, vel, pos);
        }
    }

    /**
     * this is a manual scenario: start at rest, end in motion, with quite low
     * constraints.
     */
    // @Test
    public void testStart() {
        System.out.println("START");
        MotionState start = new MotionState(0, 0, 0);
        MotionState goal = new MotionState(Double.MAX_VALUE, 3, 0); // go infinitely far
        int maxVel = 3; // m/s <== the goal vel and max vel are the same
        int maxAccel = 6; // m/s/s
        int maxJerk = 12; // m/s/s/s
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                goal,
                maxVel,
                maxAccel,
                maxJerk);
        List<MotionSegment> segments = profile.getSegments();
        assertEquals(-1, profile.duration());
        assertEquals(1, segments.size());

        System.out.printf("time, jerk, accel, vel, pos\n");
        for (double t = 0; t < profile.duration(); t += 0.1) {
            MotionState s = profile.get(t);
            double jerk = s.getJ();
            double accel = s.getA();
            double vel = s.getV();
            double pos = s.getX();
            System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n", t, jerk, accel, vel, pos);
        }
    }



    /*
     * try shifting the derivatives down
     */
    @Test
    public void testEndShift() {
        System.out.println("END SHIFT");
        // here the start and goal states are shifted by one
        // so what we say is x is actually v, what we says is v is actually a
        // and what we say is a is actually j.  and then we say j is zero.
        MotionState start = new MotionState(3, 0, 0);
        MotionState goal = new MotionState(0, 0, 0);
        int maxVel = 3; // m/s
        int maxAccel = 2; // m/s/s // observed at houston
        int maxJerk = 4; // m/s/s/s  // guess
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                goal,
                maxAccel, // note shift
                maxJerk, // note shift
                0); // just 3 degrees not 4
        List<MotionSegment> segments = profile.getSegments();
        // ??
        assertEquals(2.0, profile.duration(), 0.01);
        // ??
        assertEquals(3, segments.size());

        // now transcribe the segments back into real profiles
        List<MotionSegment> realSegments = new ArrayList<MotionSegment>();
        MotionSegment prevRealSegment = null;
        for (MotionSegment segment : segments) {
            MotionState state = segment.getStart();
            double x = 0;
            if (prevRealSegment != null) {
                MotionState end = prevRealSegment.end();
                x = end.getX();
            }
            MotionState realState = new MotionState(x, state.getX(), state.getV(), state.getA());
            double dt = segment.getDt();
            MotionSegment realSegment = new MotionSegment(realState, dt);
            prevRealSegment = realSegment;
            realSegments.add(realSegment);
            System.out.println(realSegment);
        }
        MotionProfile realProfile = new MotionProfile(realSegments);



        System.out.printf("time, jerk, accel, vel, pos\n");
        for (double t = 0; t < realProfile.duration(); t += 0.02) { // rio granularity
            MotionState s = realProfile.get(t);
            double jerk = s.getJ();
            double accel = s.getA();
            double vel = s.getV();
            double pos = s.getX();
            System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n", t, jerk, accel, vel, pos);
        }
    }

    @Test
    public void testStartShift() {
        System.out.println("START SHIFT");
        // here the start and goal states are shifted by one
        // so what we say is x is actually v, what we says is v is actually a
        // and what we say is a is actually j.  and then we say j is zero.
        MotionState start = new MotionState(0, 0, 0);
        MotionState goal = new MotionState(3, 0, 0);
        int maxVel = 3; // m/s
        int maxAccel = 2; // m/s/s // observed at houston
        int maxJerk = 4; // m/s/s/s  // guess
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                goal,
                maxAccel, // note shift
                maxJerk, // note shift
                0); // just 3 degrees not 4
        List<MotionSegment> segments = profile.getSegments();
        // ??
        assertEquals(2.0, profile.duration(), 0.01);
        // ??
        assertEquals(3, segments.size());

        // now transcribe the segments back into real profiles
        List<MotionSegment> realSegments = new ArrayList<MotionSegment>();
        MotionSegment prevRealSegment = null;
        for (MotionSegment segment : segments) {
            MotionState state = segment.getStart();
            double x = 0;
            if (prevRealSegment != null) {
                MotionState end = prevRealSegment.end();
                x = end.getX();
            }
            MotionState realState = new MotionState(x, state.getX(), state.getV(), state.getA());
            double dt = segment.getDt();
            MotionSegment realSegment = new MotionSegment(realState, dt);
            prevRealSegment = realSegment;
            realSegments.add(realSegment);
            System.out.println(realSegment);
        }
        MotionProfile realProfile = new MotionProfile(realSegments);

        System.out.printf("time, jerk, accel, vel, pos\n");
        for (double t = 0; t < realProfile.duration(); t += 0.02) { // rio granularity
            MotionState s = realProfile.get(t);
            double jerk = s.getJ();
            double accel = s.getA();
            double vel = s.getV();
            double pos = s.getX();
            System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n", t, jerk, accel, vel, pos);
        }
    }

    @Test
    public void testEndshiftLowA() {
        System.out.println("END SHIFT LOW A");
        // here the start and goal states are shifted by one
        // so what we say is x is actually v, what we says is v is actually a
        // and what we say is a is actually j.  and then we say j is zero.
        MotionState start = new MotionState(3, 0, 0);
        MotionState goal = new MotionState(0, 0, 0);
        // max accel much lower
        int maxVel = 3; // m/s
        int maxAccel = 1; // m/s/s // very low
        int maxJerk = 2; // m/s/s/s
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                goal,
                maxAccel,
                maxJerk,
                0);
        List<MotionSegment> segments = profile.getSegments();
        // in this case there should be 3 segments: max j, cruise a, and -max j,
        // because max a is not reached.
        assertEquals(3.5, profile.duration(), 0.01);
        // five segments??
        assertEquals(3, segments.size());

        // now transcribe the segments back into real profiles
        List<MotionSegment> realSegments = new ArrayList<MotionSegment>();
        MotionSegment prevRealSegment = null;
        for (MotionSegment segment : segments) {
            MotionState state = segment.getStart();
            double x = 0;
            if (prevRealSegment != null) {
                MotionState end = prevRealSegment.end();
                x = end.getX();
            }
            MotionState realState = new MotionState(x, state.getX(), state.getV(), state.getA());
            double dt = segment.getDt();
            MotionSegment realSegment = new MotionSegment(realState, dt);
            prevRealSegment = realSegment;
            realSegments.add(realSegment);
            System.out.println(realSegment);
        }
        MotionProfile realProfile = new MotionProfile(realSegments);

        System.out.printf("time, jerk, accel, vel, pos\n");
        for (double t = 0; t < realProfile.duration(); t += 0.1) {
            MotionState s = realProfile.get(t);
            double jerk = s.getJ();
            double accel = s.getA();
            double vel = s.getV();
            double pos = s.getX();
            System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n", t, jerk, accel, vel, pos);
        }
    }

}
