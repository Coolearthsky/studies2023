package glc.glc_interface;

import glc.glc_interpolation.InterpolatingPolynomial;

/**
 * \brief The user must define the infeasible space for the problem instance to inform the algorithm whether or not a trajectory is feasible
 */
public abstract class Obstacles{
      //! \brief collision_counter monitors the number of times the collisionFree method is called in a planning query
      int collision_counter=0;
      /**
       * \brief The user must implement the collisionFree method for their problem instance
       * \param[in] traj_ is the state trajectory that will be checked for intersection with the infeasible region for a particular problem
       * \returns The method returns true of the trajectory remains in the feasible region (i.e. it is collision free) and false otherwise
       */
      public abstract boolean collisionFree(final InterpolatingPolynomial traj_); 
    };