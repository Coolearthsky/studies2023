package org.team100.frc2023;

import java.util.ArrayList;
import java.util.List;

import javax.sound.midi.Track;

import org.team100.frc2023.commands.MoveAllAxes;
import org.team100.frc2023.commands.MoveSequence;
import org.team100.frc2023.commands.MoveTrack;
import org.team100.frc2023.instrument.Piano;
import org.team100.frc2023.kinematics.LynxArmAngles;
import org.team100.frc2023.math.LynxArmKinematics;
import org.team100.frc2023.midi.MidiReader;
import org.team100.frc2023.subsystems.Arm;
import org.team100.frc2023.util.TracksToEvents;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Contains and binds components. */
public class RobotContainer {

    private final Arm m_arm;
    private final Arm m_arm2;

    // private final Command m_mover = new MoveAllAxes(m_ntGoal, m_arm);
    private final XboxController m_controller = new XboxController(0);
    // private final Command m_manualMover = new ManualMove(m_controller, m_arm);
    // private final Command m_moveSequence = new MoveSequence(m_arm);
    private final Command m_autoCommand = new PrintCommand("auto goes here later");

    public RobotContainer() {
        // calibrating servos.
        // don't use +/- pi/2 for calibration, it's in the saturation region.
        // these are for arm "ONE".
        // each arm is different. TODO: multiple configs.
        LynxArmAngles.Config config = new LynxArmAngles.Config();
        config.swingCenter = 0.47;
        config.swingScale = 3.2;
        config.boomCenter = 0.48;
        config.boomScale = 4;
        config.stickOffset = 0.05;
        config.stickScale = 3.4;
        config.wristCenter = 0.55;
        config.wristScale = 3.3;

        LynxArmAngles.Factory factory = new LynxArmAngles.Factory(config);

        m_arm = new Arm(factory, Arm.PWMPorts.MAIN);
        m_arm2 = new Arm(factory, Arm.PWMPorts.MXP);

        // these are for calibration
        new JoystickButton(m_controller, XboxController.Button.kA.value).whileTrue(
                new MoveAllAxes(() -> factory.fromRad(0, -0.8, Math.PI / 2, 0, 0.5, 0.9), m_arm));
        new JoystickButton(m_controller, XboxController.Button.kB.value).whileTrue(
                new MoveAllAxes(() -> factory.fromRad(0, -0.8, Math.PI / 6, 0, 0.5, 0.9), m_arm));
        new JoystickButton(m_controller, XboxController.Button.kX.value).whileTrue(
                new MoveAllAxes(() -> factory.fromRad(0, -0.8, 5 * Math.PI / 6, 0, 0.5, 0.9), m_arm));
        new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value).whileTrue(
                new MoveAllAxes(() -> factory.fromRad(0, -0.8, Math.PI / 3, 0, 0.5, 0.9), m_arm));
        new JoystickButton(m_controller, XboxController.Button.kRightBumper.value).whileTrue(
                new MoveAllAxes(() -> factory.fromRad(0, -0.8, 2 * Math.PI / 3, 0, 0.5, 0.9), m_arm));

        LynxArmKinematics k = new LynxArmKinematics(factory, 0.148, 0.185, 0.1);
        MidiReader mr = new MidiReader();
        Track[] tracks = mr.getTracks();
        Piano piano = new Piano();
        //TracksToEvents tte = new TracksToEvents(piano, piano.get(60), k);

        MoveTrack mt = new MoveTrack(m_arm, tracks[0], piano, piano.get(60), k);

       // List<MoveSequence.Event> events = tte.toEvents(tracks);

        //List<MoveSequence.Event> events = new ArrayList<>();

        //MoveSequence eventSequence = new MoveSequence(m_arm, events);

        new JoystickButton(m_controller,
                XboxController.Button.kY.value).whileTrue(mt);


        // m_arm.setDefaultCommand(new MoveAllAxes(() -> Arm.initial, m_arm));

    }

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }

    /**
     * when a human plays the piano, they move their wrist pivot forward and up
     * to reach the black notes but the action is in the fingers.
     * the wrist is used to control dynamics, the forearm never moves
     * except for large movements, when it provides more clearance.
     * the resting position is *on* the keyboard, not above it, though for the
     * robot some clearance would give more room for inaccuracy.
     */
    private List<MoveSequence.Event> whiteKeys(LynxArmKinematics k) {
        List<MoveSequence.Event> events = new ArrayList<>();

        double keyWidthM = 0.0232;
        int key = 0;
        double y = 0.15;
        double z = 0.04;
        int inc = 1;
        double wristDown = Math.PI / 8;
        double stepLengthS = 0.5;

        for (int i = 0; i < 100; ++i) {
            if (key > 10)
                inc = -1;
            else if (key < -10)
                inc = 1;
            double tSec = i * stepLengthS;
            key += inc;
            double x = key * keyWidthM;
            LynxArmAngles angle = k.inverse(new Translation3d(x, y, z), 0, 0.5, 0.9);
            events.add(new MoveSequence.Event(tSec, angle));
            events.add(new MoveSequence.Event(tSec + 0.25 * stepLengthS, angle.down(wristDown)));
            events.add(new MoveSequence.Event(tSec + 0.75 * stepLengthS, angle));
        }
        return events;
    }

    /** move in a circle in the xz plane */
    private List<MoveSequence.Event> circle(LynxArmKinematics k) {
        List<MoveSequence.Event> circle = new ArrayList<>();
        for (double tSec = 0; tSec < 30; tSec += 0.1) {
            double x = 0.1 * Math.cos(tSec);
            double y = 0.2;
            double z = 0.2 + 0.1 * Math.sin(tSec);
            MoveSequence.Event event = new MoveSequence.Event(
                    tSec,
                    k.inverse(new Translation3d(x, y, z), 0, 0.5, 0.9));
            circle.add(event);
        }
        return circle;

    }

    /**
     * move in a line along the keyboard playing notes.
     * this uses the whole arm for finger action which is not what i want
     * but it does work.
     */
    private List<MoveSequence.Event> forearm(LynxArmKinematics k) {
        List<MoveSequence.Event> events = new ArrayList<>();
        double keyWidthM = 0.0232;
        int key = 0;
        double y = 0.15;
        double zup = 0.075;
        double zdown = 0;
        int inc = 1;
        for (int i = 0; i < 100; ++i) {
            if (key > 10)
                inc = -1;
            else if (key < -10)
                inc = 1;
            double tSec = i * 1.0;
            key += inc;
            double x = key * keyWidthM;
            events.add(new MoveSequence.Event(tSec, k.inverse(new Translation3d(x, y,
                    zup), 0, 0.5, 0.9)));
            events.add(new MoveSequence.Event(tSec + 0.25, k.inverse(new Translation3d(x, y, zdown), 0, 0.5, 0.9)));
            events.add(new MoveSequence.Event(tSec + 0.75, k.inverse(new Translation3d(x, y, zup), 0, 0.5, 0.9)));
        }
        return events;
    }

}