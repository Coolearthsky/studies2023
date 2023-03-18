// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;

public class GamepieceLocator extends SubsystemBase {
    DistanceSensor leftSensor;
    DistanceSensor rightSensor;

    /** Creates a new GamepieceLocator. */
    public GamepieceLocator() {
        leftSensor = new DistanceSensor(0x52, 0);
        rightSensor = new DistanceSensor(0x56, 0);
    }

    /**
     * Get the offset of the gamepiece from the center of the manipulator in millimeters
     * <p>Right side of the robot is positive</p>
     * @return offset in millimeters
     */
    public double getOffsetMillimeters() {
        return (leftSensor.getMillimeters() - rightSensor.getMillimeters()) / 2;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        leftSensor.periodic();
        rightSensor.periodic();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("offset", () -> this.getOffsetMillimeters(), null);
        builder.addDoubleProperty("left", () -> this.leftSensor.getMillimeters(), null);
        builder.addDoubleProperty("right", () -> this.rightSensor.getMillimeters(), null);
    }
}
