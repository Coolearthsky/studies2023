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
        // In simulation, the "X" button is the "C" key, and left Y is W and S
        Supplier<State> manualGoalRad = () -> new State(controller.getLeftY() * Math.PI/2, 0);
        MoveArmToGoal manualGoalCommand = MoveArmToGoal.factory(manualGoalRad, arm);
        controller.x().whileTrue(manualGoalCommand);

        // While holding Y, move the arm position to a fixed point and stop it there.
        // In simulation, the "Y" button is the "V" key.
        Supplier<State> fixedGoalRad = () -> new State(0, 0);
        MoveArmToGoal fixedGoalCommand = MoveArmToGoal.factory(fixedGoalRad, arm);
        controller.y().whileTrue(fixedGoalCommand);
    }
}
