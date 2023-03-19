package frc.robot.subsystems.DistanceSensors;

public abstract class DistanceSensor {

    public abstract void periodic();

    public abstract double getMillimeters();
    public double getCentimeters() {
        return getMillimeters() / 10;
    }
}
