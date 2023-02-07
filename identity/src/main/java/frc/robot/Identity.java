package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/*
 * Represents a specific RoboRIO, as a key for configurations.
 * 
 * The serial numbers here can be found on the label on the back: add a leading zero.
 */
public enum Identity {
    SWERVE_ONE("foo"),
    SWERVE_TWO("bar"),
    SQUAREBOT("baz"),
    CAMERA_DOLLY("biz"),
    TEAM100_2018("0313baf3"), // on my desk
    BLANK(""), // e.g. test default or simulation
    UNKNOWN(null);

    private static Map<String, Identity> identities = new HashMap<String, Identity>();

    static {
        for (Identity i : Identity.values()) {
            identities.put(i.m_serialNumber, i);
        }
    }

    private String m_serialNumber;

    private Identity(String serialNumber) {
        m_serialNumber = serialNumber;
    }

    public static Identity get() {
        String serialNumber = RobotController.getSerialNumber();
        if (identities.containsKey(serialNumber))
            return identities.get(serialNumber);
        return UNKNOWN;
    }

    /*
     * For simulation only.
     */
    public static void set(Identity i) {
        RoboRioSim.setSerialNumber(i.m_serialNumber);
    }

}
