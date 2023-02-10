package team100.commands;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import team100.subsystems.Arm;

/**
 * Apply the input as a goal for the arm to move to; move the arm according to
 * the trapezoid constraint as long as this command runs.
 */
public class MoveArmToGoal extends ProfiledPIDCommand {

    public static MoveArmToGoal factory(Supplier<State> goalSource, Arm arm) {
        DoubleSupplier measurementSource = () -> arm.get();
        BiConsumer<Double, State> useOutput = (output, setpoint) -> arm.set(output);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);
        ProfiledPIDController controller = new ProfiledPIDController(1, 0, 0, constraints);
        controller.setTolerance(1);
        return new MoveArmToGoal(controller, measurementSource, goalSource, useOutput, arm);
    }

    private MoveArmToGoal(
            ProfiledPIDController controller,
            DoubleSupplier measurementSource,
            Supplier<State> goalSource,
            BiConsumer<Double, State> useOutput,
            Arm arm) {
        super(controller, measurementSource,
                goalSource, useOutput, arm);
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }
}
