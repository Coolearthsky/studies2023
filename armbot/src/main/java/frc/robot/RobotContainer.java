package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team100.commands.MoveArmToGoal;
import team100.subsystems.Arm;

public class RobotContainer {
    private final CommandXboxController controller;
    private final Arm arm;

    public RobotContainer() {
        controller = new CommandXboxController(0);
        arm = new Arm(0, 0);

        // While holding X, move the arm position to the controller's left Y and stop it
        // there.
        Supplier<State> manualGoal = () -> new State(controller.getLeftY(), 0);
        MoveArmToGoal manualGoalCommand = MoveArmToGoal.factory(manualGoal, arm);
        controller.x().whileTrue(manualGoalCommand);

        // While holding Y, move the arm position to a fixed point and stop it there.
        Supplier<State> fixedGoal = () -> new State(1, 0);
        MoveArmToGoal fixedGoalCommand = MoveArmToGoal.factory(fixedGoal, arm);
        controller.y().whileTrue(fixedGoalCommand);
    }
}
