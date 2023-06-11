package org.team100.lib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;

/**
 * White noise (xi or ξ or w or η) is a process that produces uncorrelated
 * samples drawn from a gaussian distribution with zero mean and the specified
 * covariance. It's not a random variable (which represents a belief about a
 * hidden state), it is the state itself. About the only thing you can do with
 * noise is integrate it (as described by Norbert Wiener, so the "Wiener
 * process") which *does* produce a normal random variable with mean of zero and
 * variance of t times the underlying distribution, representing the resulting
 * "random walk". For comparison integration of a random variable produces t
 * *squared* times the underlying distribution: integrating noise produces less
 * variance than integrating a belief.
 */
public class WhiteNoiseVector<States extends Num> {
    public final Matrix<States, States> P;

    public WhiteNoiseVector(Matrix<States, States> P) {
        this.P = P;
    }
}
