package glc;

import java.util.Vector;

public class GlcMath {

    /**
     * \brief Computes the square of a floating point number
     * \param x_ a double that is squared
     * \return The square of x
     */
    public static double sqr(final double x) {
        return x * x;
    }

    /**
     * \brief Calculates the square of the L2-norm of a vector; more efficient than
     * sqr(norm2(x))
     * \param x_ is the input vector whose norm be computed and squared.
     * \returns The square of the norm of the parameter x.
     */

    public static double normSqr(final double[] x) {
        double norm = 0;
        for (int i = 0; i < x.length; i++) {
            norm += sqr(x[i]);
        }
        return norm;
    }

    /**
     * \brief Element-wise floor operation
     * \param x_ is the input vector whose entries will be rounded down
     */

    public static Vector<Integer> vecFloor(final double[] x) {
        Vector<Integer> floored = new Vector<Integer>(x.length);
        for (int i = 0; i < x.length; i++) {
            floored.set(i, (int) Math.floor(x[i]));
        }
        return floored;
    }

    /**
     * \brief The dot product of two vectors x and y
     * \param[in] x_ is one of the vectors in the product
     * \param[in] y_ the other vector in the product
     * \returns The dot product obtained by x'y
     */

    public static double dot(final double[] x, final double[] y) {
        if (x.length != y.length)
            throw new IllegalArgumentException();
        double z = 0;
        for (int i = 0; i < y.length; i++) {
            z += x[i] * y[i];
        }
        return z;
    }

    /**
     * \brief This method computes the l2 norm of a vector x
     * \param[in] x_ is the vector whose norm will be computed
     * \returns The return value is the l2 norm of the parameter x
     */
    public static double norm2(final double[] x) {
        double norm = 0;
        for (int i = 0; i < x.length; i++) {
            norm += sqr(x[i]);
        }
        return Math.sqrt(norm);
    }

    /**
     * \brief This method computes uniformly spaced points on an interval
     * \param[in] start_ is the lower bound of the interval
     * \param[in] end_ is the upper bound of the interval
     * \param[in] num_points_ is the number of points uniformly spaced on
     * [start_,end_)
     * \returns An array of points with length num_points_ starting at start_ and
     * ending at end_ the rightmost boundary is not included.
     */
    public static double[] linearSpace(final double start, final double end, final int points) {
        if (end <= start)
            throw new IllegalArgumentException("[ERROR] in linearSpace -- end is less than start");
        double[] lin_space = new double[points];
        double step = (end - start) / (double) points;
        lin_space[0] = start;
        for (int i = 1; i < points; i++) {
            lin_space[i] = lin_space[i - 1] + step;
        }
        return lin_space;
    }

}