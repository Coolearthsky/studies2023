package org.team100.frc2023.commands;

import java.util.function.DoubleSupplier;

import org.team100.frc2023.kinematics.LynxArmAngles;
import org.team100.frc2023.motor.ProfiledServo;
import org.team100.frc2023.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


/** Moves the arm incrementally using the Xbox controller. */
public class ManualMove extends Command {
    private static double kDt = 0.02;
    // private static double kScale = 0.1;
    private static double kScale = 1;
    private static double kDeadband = 0.05;

    private final LynxArmAngles.Factory m_factory;
    private final XboxController m_controller;
    private final Arm m_arm;

    private LynxArmAngles m_goals;

    public ManualMove(LynxArmAngles.Factory factory, XboxController controller, Arm arm) {
m_factory = factory;
        m_controller = controller;
        m_arm = arm;
        m_goals = m_factory.from0_1(0, 0, 0, 0, 0, 0);
        addRequirements(m_arm);
    }

    /** Adjusts the goals and moves one timestep. */
    @Override
    public void execute() {
        // suggested key bindings in comments; see simgui-ds.json
        m_goals = m_factory.from0_1(
                fix(m_arm.swing, m_controller::getLeftX), // sim: AD
                fix(m_arm.boom, m_controller::getLeftY), // sim: WS
                fix(m_arm.stick, m_controller::getRightY), // sim: IK
                fix(m_arm.wrist, m_controller::getRightX), // sim: JL
                fix(m_arm.twist, () -> 2 * m_controller.getRightTriggerAxis() - 1),
                fix(m_arm.grip, () -> 2 * m_controller.getLeftTriggerAxis() - 1));

        // setGoal(Arm.Axis.Swing, m_controller::getLeftX);
        // setGoal(Arm.Axis.Boom, m_controller::getLeftY);
        // etGoal(Arm.Axis.Stick, m_controller::getRightY);
        // setGoal(Arm.Axis.Wrist, m_controller::getRightX); // sim: JL
        // setGoal(Arm.Axis.Twist, ()->2*m_controller.getRightTriggerAxis()-1); // no
        // control for twist
        // setGoal(Arm.Axis.Grip, () ->2* m_controller.getLeftTriggerAxis()-1 // sim: F
        // ); // sim: H

        m_arm.setGoals(m_goals);
        //m_arm.move(kDt);
    }

    /** Reads the current position and increments it using the supplied input. */
    // private void setGoal(Arm.Axis axis, DoubleSupplier input) {

    // m_goals.put(axis, m_arm.getPosition(axis)
    // + MathUtil.applyDeadband(kScale * input.getAsDouble(), kDeadband));
    // }

    private double fix(ProfiledServo servo, DoubleSupplier input) {
        return servo.getPosition() + MathUtil.applyDeadband(kScale * input.getAsDouble(), kDeadband);

    }
}