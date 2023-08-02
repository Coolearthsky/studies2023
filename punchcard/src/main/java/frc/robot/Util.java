package frc.robot;

public class Util {
    /** @return binary representation of channel */
    public static boolean[] toArray(int channel, int maxChannels) {
        boolean[] result = new boolean[maxChannels];
        for (int i = 0; i < maxChannels; ++i) {
            result[i] = (channel & (1 << i)) != 0;
        }
        return result;
    }

    /** @return int representation of array */
    public static int toChannel(boolean[] array) {
        int result = 0;
        for (int i = 0; i < array.length; ++i) {
            result = result | ((array[i] ? 1 : 0) << i);
        }
        return result;
    }
}
