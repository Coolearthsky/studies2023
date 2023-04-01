package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * To avoid threading issues, fast subsystems don't do any work in periodic.
 */
public class FastSubsystem extends SubsystemBase {

    @Override
    public final void periodic() {
    }

    @Override
    public final void simulationPeriodic() {
    }
}
