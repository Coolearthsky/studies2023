package team100;

import edu.wpi.first.math.filter.LinearFilter;

public class Smoother {
    LinearFilter f = LinearFilter.singlePoleIIR(0.06, 0.02);

    public double calculate(double input) {
        return f.calculate(input);
    }
}
