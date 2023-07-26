package org.team100.frc2023;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2023.commands.MoveAllAxes;
import org.team100.frc2023.commands.MoveSequence;
import org.team100.frc2023.kinematics.LynxArmAngles;
import org.team100.frc2023.math.LynxArmKinematics;
import org.team100.frc2023.subsystems.Arm;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Contains and binds components. */
public class RobotContainer {
    private final Arm m_arm = new Arm();
    // private final Command m_mover = new MoveAllAxes(m_ntGoal, m_arm);
    private final XboxController m_controller = new XboxController(0);
    // private final Command m_manualMover = new ManualMove(m_controller, m_arm);
    // private final Command m_moveSequence = new MoveSequence(m_arm);
    private final Command m_autoCommand = new PrintCommand("auto goes here later");

    public RobotContainer() {
        configureButtonBindings();
        // m_arm.setDefaultCommand(m_manualMover);
    }

    private void configureButtonBindings() {
        // Moves to the spot specified in the network tables.
        // MXP
        // new JoystickButton(m_controller, XboxController.Button.kA.value).whileTrue(
        // new MoveAllAxes(() -> new LynxArmAngles(0.5, 0.75, 0.25, 0.25, 0.5, 0.5),
        // m_arm));
        // new JoystickButton(m_controller, XboxController.Button.kB.value).whileTrue(
        // new MoveAllAxes(() -> new LynxArmAngles(0.6, 0.6, 0.25, 0.6, 0.6, 0.6),
        // m_arm));
        // new JoystickButton(m_controller, XboxController.Button.kX.value).whileTrue(
        // new MoveAllAxes(() -> new LynxArmAngles(0.6, 0.6, 0.25, 0.6, 0.6, 0.6),
        // m_arm));
        // new JoystickButton(m_controller, XboxController.Button.kY.value).whileTrue(
        // new MoveAllAxes(() -> new LynxArmAngles(0.6, 0.6, 0.25, 0.6, 0.6, 0.6),
        // m_arm));

        // onboard
        // swing: 0.5 is mid, 0.25 is 45 left.
        // boom: 0.5 is mid, 0.75 is 60 back (?), 0.25 is 60 fwd
        // stick: higher is straighter, basically 0 is straight and 1 is bent
        // wrist: 0.55 is in line with the stick, roughly, 0.3 is 45 down
        // twist: 0.5 is horizontal, leave it there.
        // grip: 1 is shut, 0 is open, it's misindexed a little, should grip harder
        // up
        new JoystickButton(m_controller, XboxController.Button.kA.value).whileTrue(
                new MoveAllAxes(() -> new LynxArmAngles(0.5, 0.8, 0.9, 0.6, 0.5, 0.9), m_arm));
        // down
        new JoystickButton(m_controller, XboxController.Button.kB.value).whileTrue(
                new MoveAllAxes(() -> new LynxArmAngles(0.5, 0.8, 0.9, 0.5, 0.5, 0.9), m_arm));
        // up
        new JoystickButton(m_controller, XboxController.Button.kX.value).whileTrue(
                new MoveAllAxes(() -> new LynxArmAngles(0.25, 0.61, 0.795, 0.73, 0.5, 0.9), m_arm));
        // down
        // new JoystickButton(m_controller, XboxController.Button.kY.value).whileTrue(
        // new MoveAllAxes(() -> new LynxArmAngles(0.25, 0.61, 0.795, 0.63, 0.5, 0.9),
        // m_arm));

        // move in a circle in the xz plane
        LynxArmKinematics k = new LynxArmKinematics(0.148, 0.185, 0.1);
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

        // move in a line along the keyboard playing notes.
        // this uses the whole arm for finger action which is not what i want
        // but it does work.
        List<MoveSequence.Event> events = new ArrayList<>();
        double keyWidthM = 0.0232;
        int key = 0;
        double y = 0.15;
        double zup = 0.075;
        double zdown = 0;
        int inc = 1;
        for (int i = 0; i < 100; ++i) {
            if (key > 10) inc = -1;
            else if (key < -10) inc = 1;
            double tSec = i * 1.0;
            key += inc;
            double x = key * keyWidthM;
            events.add(new MoveSequence.Event( tSec,  k.inverse(new Translation3d(x, y, zup), 0, 0.5, 0.9)));
            events.add(new MoveSequence.Event( tSec + 0.25,  k.inverse(new Translation3d(x, y, zdown), 0, 0.5, 0.9)));
            events.add(new MoveSequence.Event( tSec + 0.75,  k.inverse(new Translation3d(x, y, zup), 0, 0.5, 0.9)));
        }

        new JoystickButton(m_controller,
                XboxController.Button.kY.value).whileTrue(new MoveSequence(m_arm, events));

        // play a couple of notes
        // new JoystickButton(m_controller, XboxController.Button.kY.value).whileTrue(
        // new MoveSequence(m_arm,
        // List.of(
        // // note 1
        // new MoveSequence.Event(0.00, new LynxArmAngles(0.25, 0.61, 0.795, 0.73, 0.5,
        // 0.9)),
        // new MoveSequence.Event(0.25, new LynxArmAngles(0.25, 0.61, 0.795, 0.63, 0.5,
        // 0.9)),
        // new MoveSequence.Event(0.50, new LynxArmAngles(0.25, 0.61, 0.795, 0.73, 0.5,
        // 0.9)),
        // new MoveSequence.Event(0.75, new LynxArmAngles(0.25, 0.61, 0.795, 0.63, 0.5,
        // 0.9)),
        // new MoveSequence.Event(1.00, new LynxArmAngles(0.25, 0.61, 0.795, 0.73, 0.5,
        // 0.9)),
        // new MoveSequence.Event(1.25, new LynxArmAngles(0.25, 0.61, 0.795, 0.63, 0.5,
        // 0.9)),
        // new MoveSequence.Event(1.50, new LynxArmAngles(0.25, 0.61, 0.795, 0.73, 0.5,
        // 0.9)),
        // new MoveSequence.Event(1.75, new LynxArmAngles(0.25, 0.61, 0.725, 0.73, 0.5,
        // 0.9)),// stick up
        // // note 2
        // new MoveSequence.Event(2.00, new LynxArmAngles(0.50, 0.80, 0.900, 0.60, 0.5,
        // 0.9)),
        // new MoveSequence.Event(2.25, new LynxArmAngles(0.50, 0.80, 0.900, 0.50, 0.5,
        // 0.9)),
        // new MoveSequence.Event(2.50, new LynxArmAngles(0.50, 0.80, 0.900, 0.60, 0.5,
        // 0.9)),
        // new MoveSequence.Event(2.75, new LynxArmAngles(0.50, 0.80, 0.900, 0.50, 0.5,
        // 0.9)),
        // new MoveSequence.Event(3.00, new LynxArmAngles(0.50, 0.80, 0.900, 0.60, 0.5,
        // 0.9)),
        // new MoveSequence.Event(3.25, new LynxArmAngles(0.50, 0.80, 0.900, 0.50, 0.5,
        // 0.9)),
        // new MoveSequence.Event(3.50, new LynxArmAngles(0.50, 0.80, 0.900, 0.60, 0.5,
        // 0.9)),
        // new MoveSequence.Event(3.75, new LynxArmAngles(0.50, 0.80, 0.725, 0.60, 0.5,
        // 0.9)),// stick up
        // // note 1 again
        // new MoveSequence.Event(4.00, new LynxArmAngles(0.25, 0.61, 0.795, 0.73, 0.5,
        // 0.9)),
        // new MoveSequence.Event(4.25, new LynxArmAngles(0.25, 0.61, 0.795, 0.63, 0.5,
        // 0.9)),
        // new MoveSequence.Event(4.50, new LynxArmAngles(0.25, 0.61, 0.795, 0.73, 0.5,
        // 0.9)),
        // new MoveSequence.Event(4.75, new LynxArmAngles(0.25, 0.61, 0.795, 0.63, 0.5,
        // 0.9)),
        // new MoveSequence.Event(5.00, new LynxArmAngles(0.25, 0.61, 0.795, 0.73, 0.5,
        // 0.9)),
        // new MoveSequence.Event(5.25, new LynxArmAngles(0.25, 0.61, 0.795, 0.63, 0.5,
        // 0.9)),
        // new MoveSequence.Event(5.50, new LynxArmAngles(0.25, 0.61, 0.795, 0.73, 0.5,
        // 0.9)))));

        // Plays a sequence of preset moves.
        // new JoystickButton(m_controller,
        // XboxController.Button.kB.value).whileTrue(m_moveSequence);
    }

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }
}