package examples;

import java.util.ArrayList;

import glc.GlcMath;
import glc.glc_interface.CostFunction;
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

class ShortestPathDemo {

    ////////////////////////////////////////////////////////
    ///////// Discretization of Control Inputs///////////////
    ////////////////////////////////////////////////////////
    class ControlInputs2D extends Inputs {

public:
  //uniformly spaced points on a circle
  ControlInputs2D(int num_inputs){

    ArrayList<Double> u(2);
    for(int i=0;i<num_inputs;i++){
      u[0]=sin(2.0*i*M_PI/num_inputs);
      u[1]=cos(2.0*i*M_PI/num_inputs);
      addInputSample(u);
    }
  }
    };

    ////////////////////////////////////////////////////////
    /////////////// Goal Checking Interface//////////////////
    ////////////////////////////////////////////////////////
    class SphericalGoal implements GoalRegion {
        double goal_radius, goal_radius_sqr;
        ArrayList<Double> error;
        ArrayList<Double> x_g;
        int resolution;

public:

  SphericalGoal(final int _state_dim, 
                final double _goal_radius,
                int _resolution) {
                x_g = new ArrayList<Double>(){{_state_dim,0.0}};
                resolution = _resolution;
                goal_radius = _goal_radius
                error = new ArrayList<Double>(){{_state_dim,0.0}};
                {
                  goal_radius_sqr=glc::sqr(goal_radius);
                }

        // Returns true if traj intersects goal and sets t to the first time at which
        // the trajectory is in the goal
    @Override
    bool inGoal(final InterpolatingPolynomial traj, double time) {
      time=traj->initialTime();

      double dt=(traj->numberOfIntervals()*traj->intervalLength())/resolution;for(int i=0;i<resolution;i++){time+=dt;
        // don't need to check t0 since it was part of last traj
      error=x_g-traj->at(time);if(glc::dot(error,error)<goal_radius_sqr){return true;}}return false;
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

    ////////////////////////////////////////////////////////
    //////// Problem Specific Admissible Heuristic///////////
    ////////////////////////////////////////////////////////
    class EuclideanHeuristic extends Heuristic {
        double radius;
        ArrayList<Double> goal;

public  EuclideanHeuristic(ArrayList<Double> _goal, double _radius) {
    radius = _radius;
    goal=_goal;
    }

@Override
 public double costToGo(final ArrayList<Double> state) {
    return Math.max(0.0,sqrt(GlcMath.sqr(goal[0]-state[0])+GlcMath.sqr(goal[1]-state[1]))-radius);
    //offset by goal radius
  }

  void setGoal(final ArrayList<Double> goal_){
    goal = goal_;
  }
};

////////////////////////////////////////////////////////
///////////////// Dynamic Model//////////////////////////
////////////////////////////////////////////////////////
class SingleIntegrator extends RungeKuttaTwo {

    SingleIntegrator(final double max_time_step_) {
        super(0.0, max_time_step_, 2);
    }

    @Override
    public ArrayList<Double> flow(
            final ArrayList<Double> x,
            final ArrayList<Double> u) {
        return u;
    }

    @Override
    public double getLipschitzConstant() {
        return 0.0;
    }
};

        ////////////////////////////////////////////////////////
        ///////////////// Cost Function//////////////////////////
        ////////////////////////////////////////////////////////
        class ArcLength extends CostFunction {
            double sample_resolution;

  ArcLength(int _sample_resolution){
     super(0.0);
     sample_resolution = _sample_resolution;
    }

  @Override
 public double cost(final InterpolatingPolynomial traj, 
              final InterpolatingPolynomial control, 
              double t0, 
              double tf)  {

      double c =0;
                double t = traj.initialTime();
                double dt = (tf-t0)/sample_resolution;
                for(int i=0;i<sample_resolution;i++){
                  c+=GlcMath.norm2(traj->at(t+dt)-traj->at(t));
                  t+=dt;
                }
                return c;
              }
        };

        ////////////////////////////////////////////////////////
        ///////////////// State Constraints//////////////////////
        ////////////////////////////////////////////////////////
        class PlanarDemoObstacles extends Obstacles {
            int resolution;
            ArrayList<Double> center1;
            ArrayList<Double> center2;

  PlanarDemoObstacles(int _resolution) {
    resolution = _resolution;
    center1({3.0,2.0});
    center2({6.0,8.0});
}

      @Override
      public boolean collisionFree(final InterpolatingPolynomial traj) {
        double t=traj.initialTime();
        double dt=(traj.numberOfIntervals()*traj.intervalLength())/resolution;
        ArrayList<Double>state;
        for(int i=0;i<resolution;i++){t+=dt;
            // don't   need to check t0 since it  was part of last traj
        state=traj->at(t);if(GlcMath.normSqr(state-center1)<=4.0 or GlcMath.normSqr(state-center2)<=4.0){return false;}}return true;
      }
        };

int main() 
{

  //Motion planning algorithm parameters
  GlcParameters alg_params;
  alg_params.res=16;
  alg_params.control_dim = 2;
  alg_params.state_dim = 2;
  alg_params.depth_scale = 100;
  alg_params.dt_max = 5.0;
  alg_params.max_iter = 50000;
  alg_params.time_scale = 20;
  alg_params.partition_scale = 40;
  alg_params.x0 = ArrayList<Double>({0.0,0.0});
  
  //Create a dynamic model
  SingleIntegrator dynamic_model(alg_params.dt_max);
  
  //Create the control inputs
  ControlInputs2D controls(alg_params.res);
  
  //Create the cost function
  ArcLength performance_objective(4);
  
  //Create instance of goal region
  ArrayList<Double> xg({10.0,10.0});

  SphericalGoal goal(xg.size(),0.25,4);
  goal.setGoal(xg);
  
  //Create the obstacles
  PlanarDemoObstacles obstacles(4);
  
  //Create a heuristic for the current goal
  EuclideanHeuristic heuristic(xg,goal.getRadius());
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
    solution->printSpline(20, "Solution");
    glc::trajectoryToFile("shortest_path_demo.txt","./",solution,500);
    glc::nodesToFile("shortest_path_demo_nodes.txt","./",planner.partition_labels);
  }
  return 0;
}

}