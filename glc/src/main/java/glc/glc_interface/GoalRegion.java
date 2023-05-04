package glc.glc_interface;

import glc.glc_interpolation.InterpolatingPolynomial;

/**
 * \brief The user must define a goal region for the problem instance which
 * informs the algorithm whether or not a trajecoty intersects the goal
 */
public interface GoalRegion {
  /**
   * \brief The user must implement the inGoal method which answers whether traj_
   * intersects the goal and if so, the earliest time at which this happens
   * \param[in] traj_ is the state trajectory that will be checked for
   * intersection with the goal region
   * \param[out] intersection_time_ is the the earliest instant at which the
   * trajectory is in the goal region
   * \returns The method returns true if traj_ intersects the goal and false
   * otherwise
   */
  boolean inGoal(final InterpolatingPolynomial traj_, double intersection_time_);
};