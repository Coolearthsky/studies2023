package team100;

import java.util.Arrays;

import edu.wpi.first.wpilibj.util.WPILibVersion;

/** Something the camera sees. */
public class Blip {
    public final int id;
    public final double[][] pose_R;
    // pose_t is a 2d array with dimension 3x1
    public final double[][] pose_t;

    protected Blip() {
        this.id = 0;
        this.pose_R = new double[3][3];
        this.pose_t = new double[3][1];
    }
    @Override
    public String toString() {
        return "Blip [id=" + id + ", pose_R=" + Arrays.deepToString(pose_R) + ", pose_t=" + Arrays.deepToString(pose_t) + "]";
    }
}
