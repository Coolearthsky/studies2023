package team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class PerformanceTest {
    @Test
    public void testSolve() {
        long start = System.currentTimeMillis();
        int iterations = 1000;
        Matrix<N2,N1> B = VecBuilder.fill(0,1);
        Matrix<N2,N1> k = VecBuilder.fill(50,5);
        for (int i =0; i < iterations; ++i) {
           Matrix<N1,N1> u = B.solve(k);
           // assertEquals(5, u.get(0,0));
        }
        long end = System.currentTimeMillis();
        double et = (double)(end-start);
        System.out.printf("ET (ms) %5.3f ET per iter (ms) %5.3f\n", et, et/iterations);
        // so B.solve is 4 microseconds, not a lot.
    }
    
    @Test
    public void testExplicit() {
        long start = System.currentTimeMillis();
        int iterations = 1000;
        Matrix<N2,N1> B = VecBuilder.fill(0,1);
        Matrix<N2,N1> k = VecBuilder.fill(50,5);
        for (int i =0; i < iterations; ++i) {
           Matrix<N1,N1> u = VecBuilder.fill(k.get(1,0));
           // assertEquals(5, u.get(0,0));
        }
        long end = System.currentTimeMillis();
        double et = (double)(end-start);
        System.out.printf("ET (ms) %5.3f ET per iter (ms) %5.3f\n", et, et/iterations);
        // but the "explicit" method is 1 us, 4x faster.  :-)
    }
}
