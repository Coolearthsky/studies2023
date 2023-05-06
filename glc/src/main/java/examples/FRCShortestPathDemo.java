package examples;

import glc.GlcParameters;
import glc.Planner;
import glc.PlannerOutput;

import java.util.Vector;

import glc.GlcLogging;
import glc.GlcMath;
import glc.GlcNode;
import glc.glc_interface.CostFunction;
import glc.glc_interface.GoalRegion;
import glc.glc_interface.Heuristic;
import glc.glc_interface.Inputs;
import glc.glc_interface.Obstacles;
import glc.glc_interpolation.InterpolatingPolynomial;
import glc.glc_numerical_integration.RungeKuttaTwo;

/* This example illustrates how to interface with the glc planner.
 * One must provide:
 * 
 * 1) A finite subset of admissible control inputs
 * parameterized by a resolution so that this finite
 * so that this finite set converges to a dense subset
 * with increasing resolution. 
 * 
 * 2) A goal checking subroutine that determines
 * if a trajectory object intersects the goal set.
 * 
 * 3) An admissible heuristic that underestimates 
 * the optimal cost-to-go from every feasible state.
 * One can always use h(x)=0 for all x as a heuristic.
 * 
 * 4) A dynamic model describing the response of the 
 * system to control inputs and also a lipschitz 
 * constant for the model.
 * 
 * 5) A feasibility or collision checking function.
 * 
 * 6) A Lipschitz continuous cost running cost for
 * candidate trajectories along with a known Lipschitz
 * constant.
 */

class FRCShortestPathDemo {

    ////////////////////////////////////////////////////////
    ///////// Discretization of Control Inputs///////////////
    ////////////////////////////////////////////////////////
    public static class ControlInputs2D extends Inputs {
        private static final double kVelocity = 0.5;

        // uniformly spaced points on a circle
        public ControlInputs2D(int num_inputs) {

            double[] u = new double[2];
            for (int i = 0; i < num_inputs; i++) {
                u[0] = kVelocity * Math.sin(2.0 * i * Math.PI / num_inputs);
                u[1] = kVelocity * Math.cos(2.0 * i * Math.PI / num_inputs);
                addInputSample(u);
            }
        }
    };

    ////////////////////////////////////////////////////////
    /////////////// Goal Checking Interface//////////////////
    ////////////////////////////////////////////////////////
    public static class SphericalGoal extends GoalRegion {
        private double goal_radius;
        private double goal_radius_sqr;
        private final double[] error;
        private double[] x_g;
        private final int resolution;

        public SphericalGoal(final int _state_dim,
                final double _goal_radius,
                int _resolution) {
            x_g = new double[_state_dim];
            resolution = _resolution;
            goal_radius = _goal_radius;
            error = new double[_state_dim];

            goal_radius_sqr = GlcMath.sqr(goal_radius);
        }

        // Returns true if traj intersects goal and sets t to the first time at which
        // the trajectory is in the goal
        @Override
        public double inGoal(final InterpolatingPolynomial traj) {
            double time = traj.initialTime();

            double dt = (traj.numberOfIntervals() * traj.intervalLength()) / resolution;
            for (int i = 0; i < resolution; i++) {
                time += dt;
                // don't need to check t0 since it was part of last traj
                for (int j = 0; j < x_g.length; ++j) {
                    error[j] = x_g[j] - traj.at(time)[j];
                }
                if (GlcMath.dot(error, error) < goal_radius_sqr) {
                    return time;
                }
            }
            return -1;
        }

        void setRadius(double r) {
            goal_radius = r;
            goal_radius_sqr = r * r;
        }

        double getRadius() {
            return goal_radius;
        }

        void setGoal(double[] _x_g) {
            x_g = _x_g;
        }

        double[] getGoal() {
            return x_g;
        }
    };

    ////////////////////////////////////////////////////////
    //////// Problem Specific Admissible Heuristic///////////
    ////////////////////////////////////////////////////////
    public static class EuclideanHeuristic extends Heuristic {
        private final double radius;
        private double[] goal;

        public EuclideanHeuristic(double[] _goal, double _radius) {
            radius = _radius;
            goal = _goal;
        }

        @Override
        public double costToGo(final double[] state) {
            return Math.max(0.0, Math.sqrt(GlcMath.sqr(goal[0] - state[0]) + GlcMath.sqr(goal[1] - state[1])) - radius);
            // offset by goal radius
        }

        void setGoal(final double[] goal_) {
            goal = goal_;
        }
    };

    ////////////////////////////////////////////////////////
    ///////////////// Dynamic Model//////////////////////////
    ////////////////////////////////////////////////////////
    public static class SingleIntegrator extends RungeKuttaTwo {

        SingleIntegrator(final double max_time_step_) {
            super(0.0, max_time_step_, 2);
        }

        @Override
        public void flow(final double[] dx, final double[] x, final double[] u) {
            for (int i = 0; i < u.length; ++i) {
                dx[i] = u[i];
            }
        }

        @Override
        public double getLipschitzConstant() {
            return 0.0;
        }
    };

    ////////////////////////////////////////////////////////
    ///////////////// Cost Function//////////////////////////
    ////////////////////////////////////////////////////////
    public static class ArcLength extends CostFunction {
        private final double sample_resolution;

        public ArcLength(int _sample_resolution) {
            super(0.0);
            sample_resolution = _sample_resolution;
        }

        @Override
        public double cost(final InterpolatingPolynomial traj,
                final InterpolatingPolynomial control,
                double t0,
                double tf) {

            double c = 0;
            double t = traj.initialTime();
            double dt = (tf - t0) / sample_resolution;
            for (int i = 0; i < sample_resolution; i++) {
                double[] traj_at_t_dt = traj.at(t + dt);
                double[] traj_at_t = traj.at(t);
                double[] diff = new double[traj_at_t.length];
                for (int j = 0; j < traj_at_t.length; ++j) {
                    diff[j] = traj_at_t_dt[j] - traj_at_t[j];
                }
                c += GlcMath.norm2(diff);
                t += dt;
            }
            return c;
        }
    };

    ////////////////////////////////////////////////////////
    ///////////////// State Constraints//////////////////////
    ////////////////////////////////////////////////////////
    public static class PlanarDemoObstacles extends Obstacles {
        private static final double kDia = 0.5;
        private final int resolution;
        // private final double[] corners1;
        // private final double[] corners2;
        private final double[] center1;
        private final double[] center2;

        public PlanarDemoObstacles(int _resolution) {
            resolution = _resolution;
            // corners1 = new double[] {6.34, 1.51};
            // corners2 = new double[] {4.41, 3.98};
            center1 = new double[] { 3.0, 2.0 };
            center2 = new double[] { 6.0, 8.0 };
        }

        @Override
        public boolean collisionFree(final InterpolatingPolynomial traj) {
            double t = traj.initialTime();
            double dt = (traj.numberOfIntervals() * traj.intervalLength()) / resolution;
            double[] state;
            for (int i = 0; i < resolution; i++) {
                t += dt;
                // don't need to check t0 since it was part of last traj
                state = traj.at(t);
                double x = state[0];
                double y = state[1];

                // nodes
                if (x  < 1.43 && y - kDia < 5.49)
                    return false;
                // opponent community
                if (x + kDia > 13.18 && y -  kDia < 5.49)
                    return false;
                // opponent loading
                if (x - kDia < 3.36 && y + kDia > 5.49)
                    return false;
                // opponent loading
                if (x - kDia < 6.71 && y +  kDia > 6.75)
                    return false;

                // baseline
                if (x + kDia > 16.54)
                    return false;
                // baseline
                if (x - kDia < 0)
                    return false;
                // sideline
                if (y + kDia > 8.02)
                    return false;
                // sideline
                if (y - kDia < 0)
                    return false;
                // red charge station
                if (x + kDia > 2.98
                        && x - kDia < 4.91
                        && y + kDia > 1.51
                        && y - kDia < 3.98)
                    return false;
                // blue charge station
                if (x + kDia > 11.63
                        && x - kDia < 13.56
                        && y + kDia > 1.51
                        && y - kDia < 3.98)
                    return false;
                // example opponents
                if (x + kDia > 8.5
                        && x - kDia < 9.5
                        && y + kDia > 4.5
                        && y - kDia < 5.5)
                    return false;
                    if (x + kDia > 6.5
                    && x - kDia < 7.5
                    && y + kDia > 5.5
                    && y - kDia < 6.5)
                return false;

                // if (GlcMath.normSqr(new double[] { state[0] - center1[0], state[1] -
                // center1[1] }) <= 4.0
                // || GlcMath.normSqr(new double[] { state[0] - center2[0], state[1] -
                // center2[1] }) <= 4.0) {
                // return false;
                // }
            }
            return true;
        }
    };

    public static void main(String... args) {

        // Motion planning algorithm parameters
        GlcParameters alg_params = new GlcParameters();
        alg_params.res = 16;
        alg_params.control_dim = 2;
        alg_params.state_dim = 2;
        alg_params.depth_scale = 100;
        alg_params.dt_max = 5.0;
        alg_params.max_iter = 50000;
        alg_params.time_scale = 20;
        alg_params.partition_scale = 40;
        // starting point is at the charge station
        alg_params.x0 = new double[] { 16.179, 6.750 };

        // Create a dynamic model
        SingleIntegrator dynamic_model = new SingleIntegrator(alg_params.dt_max);

        // Create the control inputs
        ControlInputs2D controls = new ControlInputs2D(alg_params.res);

        // Create the cost function
        ArcLength performance_objective = new ArcLength(4);

        // Create instance of goal region
        // goal is the center scoring tag, actually just outside that.
        double[] xg = new double[] { 1.43, 2.748 };

        // goal radius is
        SphericalGoal goal = new SphericalGoal(xg.length, 0.25, 4);
        goal.setGoal(xg);

        // Create the obstacles
        PlanarDemoObstacles obstacles = new PlanarDemoObstacles(4);

        // Create a heuristic for the current goal
        EuclideanHeuristic heuristic = new EuclideanHeuristic(xg, goal.getRadius());
        Planner planner = new Planner(obstacles,
                goal,
                dynamic_model,
                heuristic,
                performance_objective,
                alg_params,
                controls.readInputs());

        // Run the planner and print solution
        PlannerOutput out = planner.plan();
        if (out.solution_found) {
            Vector<GlcNode> path = planner.pathToRoot(true);
            InterpolatingPolynomial solution = planner.recoverTraj(path);
            solution.printSpline(20, "Solution");
            GlcLogging.trajectoryToFile("frc_shortest_path_demo.txt", "./", solution, 500);
            GlcLogging.nodesToFile("frc_shortest_path_demo_nodes.txt", "./", planner.partition_labels.keySet());
        }
    }

}