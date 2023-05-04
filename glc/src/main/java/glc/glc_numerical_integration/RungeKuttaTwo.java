package glc.glc_numerical_integration;

import java.util.ArrayList;
import java.util.Vector;

import glc.glc_interface.DynamicalSystem;
import glc.glc_interpolation.InterpolatingPolynomial;

/**
 * \brief This class implements the sim method of the base class Dynamical
 * System with a numerical integration scheme
 * 
 * Note that the user must still provide a concrete dynamical model with flow
 * implemented
 */
public abstract class RungeKuttaTwo extends DynamicalSystem {
    // ! \brief These are temporary variables used in the Runge-Kutta 2 integration
    // method
    // TODO i think these hould just be arrays.
    ArrayList<Double> x1, x2, f0, f1, f2;
    // ! \brief This is the maximum time step that sim is allowed to use
    double max_time_step;
    // ! \brief This is a temporary variable used by the integration scheme
    double h;

    /**
     * \brief The constructor sets the member parameters of this and the base class
     */
    public RungeKuttaTwo(double lipschitz_constant_,
            double max_time_step_,
            int state_dim_) {

        super(lipschitz_constant_);
        max_time_step = max_time_step_;
        x1 = new ArrayList<Double>(state_dim_);
        x2 = new ArrayList<Double>(state_dim_);
        f0 = new ArrayList<Double>(state_dim_);
        f1 = new ArrayList<Double>(state_dim_);
        f2 = new ArrayList<Double>(state_dim_);
    }

    /**
     * This method numerically integrates the dynamics
     * @param t0 is the initial time for the simulation
     * @param tf is the final time for the simulation
     * @param x0 is the initial state for the simulation
     * @param u is the control input defined over the interval [t0,tf]
     * @returns  the trajectory satisfying the dynamic equations
     */
    @Override
    public InterpolatingPolynomial sim(double t0, double tf, final ArrayList<Double> x0,
            final InterpolatingPolynomial u) {
        if (tf <= t0)
            throw new IllegalArgumentException();

        int num_steps = (int) Math.ceil((tf - t0) / max_time_step);
        double integration_step = (tf - t0) / num_steps;
        InterpolatingPolynomial solution = new InterpolatingPolynomial(integration_step, t0, x0.size(), 4);
        solution.reserve(num_steps);
        // set initial state and time
        ArrayList<Double> state = x0;
        double time = t0;
        // InterpolatingPolynomial traj_segment;
        // integrate
        for (int i = 0; i < num_steps; i++) {
            // Use numerical integration scheme to compute a spline extending from state
            // with input u([t,t+integration_step])
            InterpolatingPolynomial traj_segment = step(state, u, time, time + integration_step);
            // add traj_segment to solution
            solution.concatenate(traj_segment);
            time += integration_step;
            state = traj_segment.at(time);
        }
        sim_counter++;
        return solution;
    }

    /**
     * This method implements one step of the Runge-Kutta 2 numerical integration method
     * @param t1 the initial time for the integration step
     * @param t2 the final time for the integration step
     * @param x0 is the initial state for the integration step
     * @return segment is a cubic approximation of the trajectory from t1 to t2
     */
    InterpolatingPolynomial step(
            final ArrayList<Double> x0,
            final InterpolatingPolynomial u,
            final double t1,
            final double t2) {
        if (t1 >= t2)
            throw new IllegalArgumentException("[ERROR]: Integration step must be positive in RungeKuttaTwo");

        h = t2 - t1;
        f0 = flow(x0, u.at(t1));
        x1.clear();
        for (int i = 0; i < f0.size(); ++i) {
            x1.add(x0.get(i) + 0.5 * h * f0.get(i));
        }
        // x1=x0+0.5*h*f0;
        f1 = flow(x1, u.at(t1 + 0.5 * h));
        x2.clear();
        for (int i = 0; i < f1.size(); ++i) {
            x2.add(x0.get(i) + h * f1.get(i));
        }
        // x2=x0+h*f1;
        f2 = flow(x2, u.at(t2));

        // Cubic interpolation between x0 and x2 with x'(t1)=f(x0,u(t0)) and
        // x'(t2)=f(x2,u(t2))
        Vector<ArrayList<Double>> cubic = new Vector<ArrayList<Double>>();
        cubic.add(x0);// t^0 term
        cubic.add(f0);// t^1 term
        ArrayList<Double> e = new ArrayList<Double>();
        for (int i = 0; i < f0.size(); ++i) {
            e.add((-2.0 * f0.get(i) + 3.0 * f1.get(i) - f2.get(i)) / h);
        }
        cubic.add(e);// t^2 term
        e = new ArrayList<Double>();
        for (int i = 0; i < f0.size(); ++i) {
            e.add((f0.get(i) - 2.0 * f1.get(i) + f2.get(i)) / (h * h));
        }
        cubic.add(e);// t^3 term
        Vector<Vector<ArrayList<Double>>> knot_point = new Vector<Vector<ArrayList<Double>>>();
        knot_point.add(cubic);
        return new InterpolatingPolynomial(knot_point, t2 - t1, t1, x0.size(), 4);
    }
}