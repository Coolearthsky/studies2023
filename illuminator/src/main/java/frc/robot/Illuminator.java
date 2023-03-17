package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Illuminator {
    private final CANSparkMax led;

    public Illuminator() {
        led = new CANSparkMax(21, MotorType.kBrushed);
        // limit is half of the max of 2a for heat-sinked parallel
        // TODO: heat-sink and boost to full output
        led.setSecondaryCurrentLimit(1); 
        led.setInverted(true);
    }

    public void on() {
        led.set(1);
    }

    public void off() {
        led.set(0);
    }
}
