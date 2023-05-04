package examples;

import java.util.ArrayList;

import glc.GlcMath;
import glc.glc_interface.CostFunction;
import glc.glc_interface.GoalRegion;
import glc.glc_interface.Heuristic;
import glc.glc_interface.Inputs;
import glc.glc_interface.Obstacles;
import glc.glc_interpolation.InterpolatingPolynomial;
import glc.glc_numerical_integration.RungeKuttaTwo;

public class NonholonomicCarDemo {

    ////////////////////////////////////////////////////////
    ///////// Discretization of Control Inputs///////////////
    ////////////////////////////////////////////////////////
    public static class ControlInputs2D extends Inputs {
        // uniformly spaced points on a circle
public  ControlInputs2D(int num_inputs){

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

        ArrayList<Double> center;

  public Sphericalgoal(double _goal_radius_sqr, 
  ArrayList<Double> _goal_center,
  int _resolution) {
  resolution = _resolution;
  center = _goal_center;
  radius_sqr = _goal_radius_sqr;}

  SphericalGoal(final int _state_dim, 
                final double _goal_radius,
                int _resolution) {

                x_g = _state_dim,0.0;
                resolution = _resolution;
                goal_radius = _goal_radius;
                error = _state_dim,0.0;
                
                  goal_radius_sqr=GlcMath.sqr(goal_radius);
                }

        // Returns true if traj intersects goal and sets t to the first time at which
        // the trajectory is in the goal
        @Override
                bool inGoal(final InterpolatingPolynomial traj, Double time)  {
                  time=traj->initialTime();
                  
                  double dt=(traj->numberOfIntervals()*traj->intervalLength())/resolution;
                  for(int i=0;i<resolution;i++){
                    time+=dt;//don't need to check t0 since it was part of last traj
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

        void setGoal(ArrayList<Double> _x_g) {
            x_g = _x_g;
        }

        ArrayList<Double> getGoal() {
            return x_g;
        }

        // Returns true if traj intersects goal and sets t to the first time at which
        // the trajectory is in the goal
        bool inGoal(final InterpolatingPolynomial traj, double time) {
            time = traj.initialTime();

            double dt = (traj.numberOfIntervals() * traj.intervalLength()) / resolution;
            for (int i = 0; i < resolution; i++) {
                time += dt;// don't need to check t0 since it was part of last traj
                ArrayList<Double> state = traj -> at(time);
                if (GlcMath.sqr(state[0] - center[0]) + GlcMath.sqr(state[1] - center[1]) < radius_sqr) {
                    return true;
                }
            }
            return false;
        }
    };

    ////////////////////////////////////////////////////////
    ///////////////// Dynamic Model//////////////////////////
    ////////////////////////////////////////////////////////
    class SingleIntegrator extends RungeKuttaTwo {
        public SingleIntegrator(final double max_time_step_) {
            super(0.0, max_time_step_, 2);
        }

        @Override
        ArrayList<Double> flow(final ArrayList<Double> x, final ArrayList<Double> u) {
            return u;
        }

        double getLipschitzConstant() {
            return 0.0;
        }
    };

    ////////////////////////////////////////////////////////
    ///////////////// Cost Function//////////////////////////
    ////////////////////////////////////////////////////////
    class ArcLength extends CostFunction {
        double sample_resolution;

        public ArcLength(int _sample_resolution) {
            super(0.0);
            sample_resolution = _sample_resolution;

        }

  @Override
  public double cost(final InterpolatingPolynomial traj, 
              final InterpolatingPolynomial control, 
              double t0, 
              double tf)  {

        double c(0);
                double t = traj->initialTime();
                double dt = (tf-t0)/sample_resolution;
                for(int i=0;i<sample_resolution;i++){
                  c+=glc::norm2(traj->at(t+dt)-traj->at(t));
                  t+=dt;
                }
                return c;
              }
    };

    class CarControlInputs extends Inputs {

        // uniformly spaced points on a circle
 public CarControlInputs(int num_steering_angles){

        // Make all pairs (forward_speed,steering_angle)
    ArrayList<Double> car_speeds({1.0});// Pure path planning

    ArrayList<Double> steering_angles = glc::linearSpace(-0.0625*M_PI,0.0625*M_PI,num_steering_angles);

    ArrayList<Double> control_input(2);
    for(double& vel : car_speeds){
      for(double& ang : steering_angles ){
        control_input[0]=vel;
        control_input[1]=ang;
        addInputSample(control_input);
      }
    }
  }
    };

    ////////////////////////////////////////////////////////
    //////// Problem Specific Admissible Heuristic///////////
    ////////////////////////////////////////////////////////
    class EuclideanHeuristic extends Heuristic {
        double radius;
        ArrayList<Double> goal;

        public EuclideanHeuristic(ArrayList<Double> _goal, double _radius) {
            radius = _radius;
            goal = _goal;
        }

        @Override
        public double costToGo(final ArrayList<Double> state) {
            return Math.max(0.0, sqrt(GlcMath.sqr(goal[0] - state[0]) + GlcMath.sqr(goal[1] - state[1])) - radius);
            // offset by goal radius
        }
    };

    ////////////////////////////////////////////////////////
    ///////////////// Dynamic Model//////////////////////////
    ////////////////////////////////////////////////////////
    public static class CarNonholonomicConstraint extends RungeKuttaTwo {
public  CarNonholonomicConstraint(final double _max_time_step) {
    super(1.0,_max_time_step,3); {
}

        @Override
        public ArrayList<Double> flow(final ArrayList<Double> x, final ArrayList<Double> u) {
            ArrayList<Double> dx = new ArrayList<Double>();
            dx.add(u[0] * cos(x[2]));
            dx.add(u[0] * sin(x[2]));
            dx.add(u[1]);
        }

        @Override
        public double getLipschitzConstant() {
            return lipschitz_constant;
        }
    };

    ////////////////////////////////////////////////////////
    ///////////////// State Constraints//////////////////////
    ////////////////////////////////////////////////////////
    public static class PlanarDemoObstacles extends Obstacles {
        int resolution;
        ArrayList<Double> center1;
        ArrayList<Double> center2;

public PlanarDemoObstacles(int _resolution) {
    resolution = _resolution;
    center1({3.0,2.0});
    center2({6.0,8.0});
}

  @Override
public  bool collisionFree(final InterpolatingPolynomial traj)  {
    double t=traj->initialTime();
    double dt=(traj->numberOfIntervals()*traj->intervalLength())/resolution;
    ArrayList<Double> state;
    for(int i=0;i<resolution;i++){
      t+=dt;//don't need to check t0 since it was part of last traj
      state=traj->at(t);
      
      //Disk shaped obstacles
      if(glcMath.sqr(state[0]-center1[0])+glc::sqr(state[1]-center1[1]) <= 4.0 or
        glcMath.sqr(state[0]-center2[0])+glc::sqr(state[1]-center2[1]) <= 4.0 )
      {
        return false;
      }
    }
    return true;
  }
    };

    ////////////////////////////////////////////////////////
    /////////////// Run a planning query in main/////////////
    ////////////////////////////////////////////////////////
int main() {
  
  //Motion planning algorithm parameters
  GlcParameters alg_params;
  alg_params.res=21;
  alg_params.control_dim = 2;
  alg_params.state_dim = 3;
  alg_params.depth_scale = 100;
  alg_params.dt_max = 5.0;
  alg_params.max_iter = 50000;
  alg_params.time_scale = 20;
  alg_params.partition_scale = 60;
  alg_params.x0 = ArrayList<Double>({0.0,0.0,M_PI/2.0});
  
  //Create a dynamic model
  CarNonholonomicConstraint dynamic_model(alg_params.dt_max);
  
  //Create the control inputs
  CarControlInputs controls(alg_params.res);
  
  //Create the cost function
  ArcLength performance_objective(4);
  
  //Create instance of goal region
  double goal_radius_sqr(.25);

  ArrayList<Double> goal_center({10.0,10.0});

  Sphericalgoal goal(goal_radius_sqr,goal_center,10);
  
  //Create the obstacles
  PlanarDemoObstacles obstacles(10);
  
  //Create a heuristic for the current goal
  EuclideanHeuristic heuristic(goal_center,sqrt(goal_radius_sqr));

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
    glc::trajectoryToFile("nonholonomic_car_demo.txt","./",solution,500);
    glc::nodesToFile("nonholonomic_car_demo_nodes.txt","./",planner.partition_labels);
  }
  return 0;
}
}