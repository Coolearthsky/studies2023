package org.team100.lib.simulation;

import org.team100.lib.reference.Reference;

import edu.wpi.first.math.numbers.N2;

public abstract class Scenario {

    abstract String label();

    abstract Reference<N2> reference();
}