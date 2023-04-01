package team100;

import org.junit.jupiter.api.Test;

public class SmootherTest {
    @Test
    public void testSmoothing() {
        Smoother s = new Smoother();
        for (double t  = 0.0; t < 2.0; t += 0.02) {
            // some shape with noise
            double input = Math.sin(t*Math.PI/2)+ Math.random()/4;
            double output = s.calculate(input);
            System.out.printf("%5.3f %5.3f %5.3f\n",t,  input, output);
        }
    }
    
}
