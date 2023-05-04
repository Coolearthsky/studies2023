package glc;

import java.util.Vector;

import glc.glc_interface.CostFunction;
import glc.glc_interface.DynamicalSystem;
import glc.glc_interface.GoalRegion;
import glc.glc_interface.Heuristic;
import glc.glc_interface.Obstacles;
import glc.glc_interpolation.InterpolatingPolynomial;


import java.util.Iterator;
import java.util.PriorityQueue;
import java.util.Set;

/**
   * The core planning class for carring out trajectory optimization
   * 
   * The algorithms is simply an A* search. The innovation is in the 
   * construction of a sparse, and nearly optimal discrete abstraction
   * of the set of feasible trajectories for a problem instance. 
   */
 public  class Planner{
    // A leaf node in the goal region with minimum cost from the root
     GlcNode best;
    // The root of the search tree
     GlcNode root_ptr;
    // A raw pointer to the dynamical system for the problem
    DynamicalSystem dynamics;
    // A raw pointer to the goal region for the problem
    GoalRegion goal;
    // A raw pointer to the (topologically)closed obstacle set for the problem
    Obstacles obs;
    // A raw pointer to the cost function for the problem
    CostFunction cf;
    // A raw pointer to the heuristic for the problem
    Heuristic h;
    // A comparator for measureing relative merit of nodes
    NodeMeritOrder compare;
    /**
     * A priority queue of leaf nodes that are open for expansion
     * 
     * The queue is ordered by the cost plus the estimated cost to go 
     * determined by the admissible and consistent heuristic.
     */ 
//    PriorityQueue< GlcNode, Vector< GlcNode>, NodeMeritOrder> queue;
// TODO initialize this with the right comparator
    PriorityQueue< GlcNode> queue = new PriorityQueue<GlcNode>(new NodeMeritOrder());
    
    /**
     * \brief A constant factor multiplying the depth limit of the search tree
     * 
     * In order to guarantee finite time termination, the algorithm is limited
     * to a finite search depth for a fixed search resolution. This limit is 
     * increased with increasing resolution at a carefully controlled rate. 
     * The user is free to tune a constant factor.
     */
    int depth_limit;
    /**
     * \brief A constant factor dilation of the equivalence class size
     *
     * The partition size is carefully controlled by the resolution 
     * parameter. It will decrease as the resolution increases, and 
     * with larger Lipschitz coefficients in the differential 
     * constraint the faster the partition must shrink to guaranteed
     * convergence. partition_scale is a constant factor multiplying
     * this function of the Lipschitz constant. 
     */
    double partition_scale;
    
    /**
     * \brief The side length of the cubicle's forming the partition of the state space
     * 
     * This value is a function of resolution determined
     * in the constructor. In the paper this value is eta(R).
     */
    double inverse_cubicle_side_length;
    /**
     * \brief When a node is expanded, each this is the duration of the trajectories to the child nodes
     */
    double expand_time;
    /**
     * \brief This is used as a flag to stop expanding nodes and clear the priority queue to find the best solution
     */
    boolean found_goal=false; 
    /**
     * \brief This is a flag to stop the algorithm if some limit has been reached
     */
    boolean live=true;
    /**
     * \brief a copy of the struct containing planner parameters
     */
    GlcParameters params;
    /**
     * \brief An array containing the discrete set of control inputs
     * 
     * When a node is expanded each of these control inputs
     * is applied to the system dynamics from the state associated
     * with the node being expanded for a duration of 
     * expand_time.
     */
    Vector<double[]> controls;
    //! \brief A counter for the number of calls to the sim method
    int sim_count = 0;
    //! \brief A counter for the number of calls to collisionFree
    int coll_check = 0;
    //! \brief A counter for the number of calls to the expand method
    int iter=0;
    //! \brief A counter for the number of clock cycles used in a query
    long run_time, tstart;

    /**
     * \brief An ordered set of equivalence classes that have been reached by a trajectory from the initial state
     * 
     * The labels on each equivalence class is the node
     * with best known merit in that equivalence class.
     */
    public Set<GlcStateEquivalenceClass> partition_labels;
    Iterator<GlcStateEquivalenceClass> it;
    
    /** 
     * \brief The constructor assigns its parameters to the associated member attributes
     * 
     * The essential scaling functions of the method are evaluated at the given
     * resolution within the constructor.
     */
    public Planner(Obstacles _obs, 
            GoalRegion _goal, 
            DynamicalSystem _dynamics, 
            Heuristic _h, 
            CostFunction _cf, 
            final GlcParameters _params, 
            final Vector<double[]> _controls) {
              
            params = _params;
            controls = _controls; 
            dynamics = _dynamics; 
            obs = _obs;
            goal = _goal; 
            cf = _cf;
            h = _h;
root_ptr = new GlcNode(_controls.size(),0, 0,_h->costToGo(params.x0),params.x0,0,null,null,null);
best = new GlcNode(0, -1, Double.MAX_VALUE, Double.MAX_VALUE,new double[0],0,null,null,null);
////////////*Scaling functions*//////////////
// 1/R
expand_time=params.time_scale/(double)params.res;
//h(R)
depth_limit=params.depth_scale*params.res*floor(log(params.res));
//eta(R) \in \little-omega (log(R)*R^L_f)
if(dynamics->getLipschitzConstant()==0.0){
inverse_cubicle_side_length = params.res*log(params.res)*log(params.res)/params.partition_scale;
}
else{
inverse_cubicle_side_length = pow(params.res,1+dynamics->getLipschitzConstant())/params.partition_scale;
}

GlcStateEquivalenceClass d0;
d0.label = root_ptr;
d0.coordinate = vecFloor(inverse_cubicle_side_length*root_ptr->state);
queue.push(root_ptr);
partition_labels.insert(d0);

//Print a summary of the algorithm parameters
System.out.println(
System.out.println("\n\n\n\nPre-search summary:\n");
System.out.println( "      Expand time: " + expand_time );
System.out.println( "      Depth limit: " + depth_limit);
System.out.println("   Partition size: " + 1.0/inverse_cubicle_side_length );
System.out.println( "   Max iterations: " + params.max_iter );

// tstart = clock();
tstart = System.currentTimeMillis() ;
}


    /**
     * \brief This method returns the sequence of nodes from the lowest cost node in the goal back to the root via edge relations stored in each node
     * \param[in] forward is a flag to indicate if the nodes should be ordered from root to leaf (foward=true) or from leaf to root (forward=false)
     */

 public Vector<GlcNode> pathToRoot(boolean forward){
  if(found_goal==false){
    Vector< GlcNode> empty_vector;
    return empty_vector;
  }
   GlcNode currentNode = best;
  Vector< GlcNode> path;
  while( not (currentNode->parent == nullptr) ){
    path.push_back(currentNode);
    currentNode=currentNode->parent;
  }
  path.push_back(currentNode);
  if(forward){std::reverse(path.begin(),path.end());}
  return path;
}
 
    /**
     * \brief Constructs the trajectory from the root to the leaf node in the goal
     * \param[in] path is the sequence of nodes from root to leaf connected by edge relations
     * \returns a pointer to a trajectory object representing the solution trajectory
     */
  public InterpolatingPolynomial recoverTraj(final Vector< GlcNode> path){
    if(path.size()<2){return null;}
    InterpolatingPolynomial opt_sol=path.get(1).trajectory_from_parent;
    for(int i=2;i<path.size();i++){
      opt_sol->concatenate(path.get(i).trajectory_from_parent);
   }
    return opt_sol;
  } 

    
    /**
     * The core iteration of the algorithm that pops the top of the queue and forward integrates the dynamics with each of the controls
     */               
 void expand(){
  //Increment the iteration count
  iter++;
  //If the queue is empty then the problem is not feasible at the current resolution
  if(queue.isEmpty()){
    System.out.println("---The queue is empty. Resolution too low or no solution at all.---");
    live=false;
    return;
  }
  
  //Pop the top of the queue for expansion
   GlcNode current_node = queue.poll();
 // queue.pop();
  
  //Once a goal node is found we clear the queue and keep the lowest cost node in the goal
  if(found_goal){
    double goal_time;
    if(goal->inGoal(current_node.trajectory_from_parent, goal_time)){
      double cost = current_node.parent.cost + 
                    cf->cost(current_node.trajectory_from_parent,
                             current_node.control_from_parent,
                             current_node.trajectory_from_paren.initialTime(),
                             goal_time);
      if(cost<best->cost){
        best = current_node;
        // run_time = clock() - tstart;
        run_time = System.currentTimeMillis() - tstart;
        live=false;
        System.out.println( "\n\nFound goal at iter: " << iter );
        System.out.println( "     solution cost: " << best->cost );
        System.out.println( "      running time: " << (float) run_time/ (float) CLOCKS_PER_SEC );
        System.out.println( "  Simulation count: " << dynamics->sim_counter );
        System.out.println( "  Collision checks: " << obs->collision_counter );
        System.out.println( "       Size of set: " << partition_labels.size() );
        System.out.println( "     Size of queue: " << queue.size() );
      }
    }
  }
  
  //Stop the algorithm if the search tree reaches the depth or iteration limit
  if(current_node->depth >=depth_limit or iter>params.max_iter)
  {
    System.out.println( "---exceeded depth or iteration limit---" );
    live=false;
    return;
  }
  
  //A set of equivalence classes visited by new nodes made by expand
  Set<GlcStateEquivalenceClass> domains_needing_update; 
  
  //Expand top of queue and store arcs in set of domains
  for(int i=0;i<controls.size();i++){
    InterpolatingPolynomial new_traj;
    double[] c0;
    //Create a control signal spline which is a first order hold.
    //u(t)=c0+c1*t. If expanding root, just use u(t)=constant;
    if(current_node->parent==nullptr){
      c0 = controls[i];
    }
    else{
      c0 = controls[current_node->u_idx];
    }
    double[] c1 = (controls[i]-c0)/expand_time;
    Vector<double[] > segment({c0,c1});
    Vector< Vector< double[] > > linear_interp({segment});
    //The above parameters are used to construct new_control
    InterpolatingPolynomial new_control(new InterpolatingPolynomial(linear_interp,expand_time,current_node->time,controls[i].size(),2));
    //Forward simulate with new_control to get a cubic spline between collocation points
    dynamics->sim(new_traj, current_node->time, current_node->time+expand_time , current_node->state, new_control);
     GlcNode new_arc(new  GlcNode(controls.size(),
                                           i,
                                           cf->cost(new_traj, new_control,current_node->time,current_node->time+expand_time)+current_node->cost, 
                                           h->costToGo(new_traj->at(current_node->time+expand_time)), 
                                           new_traj->at(current_node->time+expand_time), 
                                           current_node->time+expand_time,
                                           current_node,
                                           new_traj,
                                           new_control
                                          ));

    //Create a region for the new trajectory
    double[] w = inverse_cubicle_side_length * new_arc->state;
    GlcStateEquivalenceClass d_new;
    d_new.coordinate = vecFloor( w );
    //Get the domain for the coordinate or create it and insert into labels.
    GlcStateEquivalenceClass bucket = const_cast<StateEquivalenceClass>( *(partition_labels.insert(d_new).first) );
    //Add to a queue of domains that need inspection
    domains_needing_update.insert(bucket);
    
    if(not compare(new_arc,bucket.label)){
      bucket.candidates.push(new_arc);
    }     
  }
  //Go through the new trajectories and see if there is a possibility for relabeling before collcheck
  for(var open_domain : domains_needing_update){
    GlcStateEquivalenceClass current_domain = open_domain;
    //We go through the queue of candidates for relabeling/pushing in each set
    boolean found_best = false;
    while( (not found_best) and (not current_domain.candidates.empty()))
    {
      //If the top of the candidate queue is cheaper than the label we should coll check it
      if(not compare(current_domain.candidates.top(),current_domain.label)){
         Node best_relabel_candidate = current_domain.candidates.top(); 
        InterpolatingPolynomial candidate_traj = best_relabel_candidate->trajectory_from_parent;//traj_from_parent[best_relabel_candidate];
        if(obs->collisionFree(candidate_traj)){
          //Flag vertex if it's in the goal
          double time;
          if( goal->inGoal(candidate_traj,time)){
            found_goal = true;
          }
          queue.push(best_relabel_candidate);//anything coll free at this point goes to queue
          if(!found_best){
            found_best = true;
            current_domain.label = best_relabel_candidate;
          }
        }
      }
      current_domain.candidates.pop();
    }
    if(current_domain.empty()){
      partition_labels.erase(current_domain);
    }
  }
  return;
}

    /**
     * \brief Method that calls expand until one of several flags is set for the algorithm to stop
     */
    void expandWhileLive(){
      while(live){
        expand();
      }
      return;
    }
    


    /**
     * Overload of plan() which provides some output on the result
     * @returns  a struct containing info on whether a solution was found and what its cost is
     */
    public PlannerOutput plan(){
      expandWhileLive();
      PlannerOutput out = new PlannerOutput();
      out.cost=best.cost;
      out.time=(float) run_time/ (float) CLOCKS_PER_SEC; 
      out.solution_found=found_goal;//TODO change to found_goal
      return out;
    }


    /**
     * \brief
     */
    boolean getSolution(InterpolatingPolynomial traj_out);
    
  };





       
 

 

 


  

 