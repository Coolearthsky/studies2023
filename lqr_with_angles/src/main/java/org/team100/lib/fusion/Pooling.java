package org.team100.lib.fusion;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Num;

public interface Pooling <States extends Num> {
    RandomVector<States> fuse(RandomVector<States> a, RandomVector<States> b);
}
