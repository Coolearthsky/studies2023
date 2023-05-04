package examples;

import java.util.ArrayList;

import glc.glc_interface.CostFunction;
import glc.glc_interface.GoalRegion;
import glc.glc_interface.Heuristic;
import glc.glc_interface.Inputs;
import glc.glc_interpolation.InterpolatingPolynomial;
import glc.glc_numerical_integration.RungeKuttaTwo;

public class PendulumSwingupDemo {
    /**
     * This is an example of a discrete input set for the torque limited
     * pendulum swingup problem where the sampling of inputs from the
     * addmissible torque inputs is equal to the resolution parameter for
     * the glc algorithm.
     */
    class PendulumTorque extends Inputs {
  public    PendulumTorque(int resolution){

      ArrayList<Double> u(1);
      double max_torque=0.1;
      for(double torque=-max_torque;torque<=max_torque;torque+=2.0*max_torque/resolution){
        u[0]=torque;
        addInputSample(u);
      }
    }
    };

    /**
     * The goal region for this example is a small circular domain in
     * the angle-velocity space for the pendulum centered where
     * the pendulum is in the inverted position and the velocity is zero.
     * It is required that the goal region have a nonempty interior.
     */
    class SphericalGoal implements GoalRegion {
        double goal_radius, goal_radius_sqr;
        ArrayList<Double> error;
        ArrayList<Double> x_g;
        int num_samples;

  public    SphericalGoal(final int _state_dim, 
                  final double _goal_radius,
                  int _num_samples) {
                  x_g = _state_dim,0.0;
                  num_samples = _num_samples;
                  goal_radius = _goal_radius;
                  error = _state_dim,0.0;
                    goal_radius_sqr=GlcMath.sqr(goal_radius);
                  }

        @Override
                  bool inGoal(final InterpolatingPolynomial traj, double time)  {
                    time=traj->initialTime();
                    double dt=(traj->numberOfIntervals()*traj->intervalLength())/num_samples;
                    for(int i=0;i<num_samples;i++){
                      time+=dt;
                      error=x_g-traj->at(time);
                      if(glc::dot(error,error) < goal_radius_sqr){
                        return true;}
                    }
                    return false;
                  }

        void setRadius(double r) {
            goal_radius = r;
            goal_radius_sqr = r * r;
        }

        double getRadius() {
            return goal_radius;
        }

        void setGoal(ArrayList<Double> _x_g){
            x_g=_x_g;
        }

        ArrayList<Double> getGoal() {
            return x_g;
        }
    };

    /**
     * Since the pendulum has non-trivial dynamics, we will use the
     * zero heuristic for this example
     */
    public static class ZeroHeuristic extends Heuristic {
        public ZeroHeuristic() {
        }

        @Override
        public double costToGo(final ArrayList<Double> state) {
            return 0.0;
        }
    };

    /**
     * The dynamic model for this example is a pendulum with
     * origin defined at the stable equilibrium. The input
     * is a torque applied at the pivot.
     */
    class InvertedPendulum extends RungeKuttaTwo {
        // For the chosen coordinate system for the dynamic model, the Lipschitz
        // constant is 1.0
        public InvertedPendulum(final double max_time_step_) {
            super(1.0, max_time_step_, 2);
        }

        @Override
        public ArrayList<Double> flow(final ArrayList<Double> x, final ArrayList<Double> u) {
            ArrayList<Double> dx = new ArrayList<Double>();
            dx.add(x.get(1));
            dx.add(u.get(0) - Math.sin(x.get(0)));
            return dx;
        }

        @Override
        public double getLipschitzConstant() {
            return lipschitz_constant;
        }
    };

    /**
     * We will use the minimum time performance objective.
     */
    class MinTime extends CostFunction {
        double sample_resolution;

        public MinTime(int _sample_resolution) {
            super(0.0);
            sample_resolution = _sample_resolution;
        }

        @Override
        public double cost(final InterpolatingPolynomial traj,
                final InterpolatingPolynomial control,
                double t0,
                double tf) {
            return tf - t0;
        }
    };

    /**
     * This is an unconstrained free space all states are collision free
     */
    class PendulumStateSpace:public glc::Obstacles{
  public:

        PendulumStateSpace(int _resolution) {
        }

    @Override
    bool collisionFree(final InterpolatingPolynomial traj)  {
      if(fabs(traj.at(traj.initialTime())[0])>3.2

        or fabs(traj.at(traj.initialTime())[1])>3.2){
        return false;
      }
      return true;
    }
    };

int main() 
{

  //Motion planning algorithm parameters
  GlcParameters alg_params;
  alg_params.res=5;
  alg_params.control_dim = 1;
  alg_params.state_dim = 2;
  alg_params.depth_scale = 100;
  alg_params.dt_max = 5.0;
  alg_params.max_iter = 50000;
  alg_params.time_scale = 7;
  alg_params.partition_scale = 2.5;
  alg_params.x0 = ArrayList<Double>({0.0,0.0});
  
  //Create a dynamic model
  InvertedPendulum dynamic_model(alg_params.dt_max);
  
  //Create the control inputs
  PendulumTorque controls(alg_params.res);
  
  //Create the cost function
  MinTime performance_objective(4);
  
  //Create instance of goal region
  ArrayList<Double> xg({M_PI,0.0});

  SphericalGoal goal(xg.size(),0.25,6);
  goal.setGoal(xg);
  
  //Create the obstacles. 
  PendulumStateSpace obstacles(4);
  
  //Create a heuristic for the current goal
  ZeroHeuristic heuristic;
  
  //Construct the planner
  Planner planner(obstacles,
                       goal,
                       dynamic_model,
                       heuristic,
                       performance_objective,
                       alg_params,
                       controls.readInputs());
  
  //Run the planner and print solution
  PlannerOutput out = planner.plan();
  if(out.solution_found){
    Vector<Node> path = planner.pathToRoot(true);
    InterpolatingPolynomial solution = planner.recoverTraj( path );
    solution->printData();
    glc::trajectoryToFile("pendulum_swingup_demo.txt","./",solution,500);
  }
  glc::nodesToFile("pendulum_swingup_demo_nodes.txt","./",planner.partition_labels);
  return 0;
}
}