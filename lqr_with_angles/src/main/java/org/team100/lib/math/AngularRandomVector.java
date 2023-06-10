package org.team100.lib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** example of handling non-euclidean metrics. */
public class AngularRandomVector<States extends Num> extends RandomVector<States> {

    public AngularRandomVector(Matrix<States, N1> x, Matrix<States, States> P) {
        super(x, P);
    }

    public AngularRandomVector<States> make(Matrix<States, N1> x, Matrix<States, States> P) {
        return new AngularRandomVector<>(x,P);
    }

    /** angle version for angle in zeroth spot */
    @Override
    public Matrix<States, N1> xplus(Matrix<States, N1> otherx) {
        Matrix<States, N1> x = super.xplus(otherx);
        x.set(0, 0, MathUtil.angleModulus(x.get(0, 0)));
        return x;
    }

    /** angle version for angle in zeroth spot */
    @Override
    public Matrix<States, N1> xminus(Matrix<States, N1> other) {
        Matrix<States, N1> x = super.xminus(other);
        x.set(0, 0, MathUtil.angleModulus(x.get(0, 0)));
        return x;
    }
}
