// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DistanceSensors.DistanceSensor;
import frc.robot.subsystems.DistanceSensors.VL53L4CD;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GamepieceLocator extends SubsystemBase {
    DistanceSensor leftSensor;
    DistanceSensor rightSensor;

    /** Creates a new GamepieceLocator. */
    public GamepieceLocator() {
        leftSensor = new VL53L4CD(0x30, 0, 0);
        rightSensor = new VL53L4CD(0x32, 1, 0);
        // leftSensor = new GP2Y0A21(0, 0);
        // rightSensor = new GP2Y0A21(1, 3);

        SmartDashboard.putData("Gamepiece Locator", this);
    }

    /**
     * Get the offset of the gamepiece from the center of the manipulator in millimeters
     * <p>Right side of the robot is positive</p>
     * @return offset in millimeters
     */
    public double getOffsetMillimeters() {
        return (leftSensor.getMillimeters() - rightSensor.getMillimeters()) / 2;
    }

    /**
     * Get the offset of the gamepiece from the center of the manipulator in centimeters
     * <p>Right side of the robot is positive</p>
     * @return offset in centimeters
     */
    public double getOffsetCentimeters() {
        return getOffsetMillimeters() / 10;
    }

    /**
     * Get the offset of the gamepiece from the center of the manipulator in meters
     * <p>Right side of the robot is positive</p>
     * @return offset in meters
     */
    public double getOffsetMeters() {
        return getOffsetMillimeters() / 1000;
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
        builder.addDoubleProperty("offset", () -> this.getOffsetCentimeters(), null);
        builder.addDoubleProperty("left", () -> this.leftSensor.getCentimeters(), null);
        builder.addDoubleProperty("right", () -> this.rightSensor.getCentimeters(), null);
    }
}
