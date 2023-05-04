package glc;

import java.util.ArrayList;
import glc.glc_interpolation.InterpolatingPolynomial;

/**
 * \brief Node object used in motion planning search tree
 * 
 * The Node object in the glc library will always belong to a motion
 * planning search tree. Each Node contains pointers to its parent
 * and children in the tree. A Node is also associated to a state in
 * the state space as well as a cost to reach that Node via the path
 * from the root of the tree. The node also has an attribute "merit"
 * which is the cost from the root together with the estimated
 * cost-to-go which is used in the A*-like search.
 */
class GlcNode {
  // ! \brief The pointer to this node's parent
  GlcNode parent;
  // ! \brief The trajectory from the parent node to this node
  InterpolatingPolynomial trajectory_from_parent;
  // ! \brief The control signal producing the trajectory from the parent node to
  // this node
  InterpolatingPolynomial control_from_parent;
  // ! \brief An array of pointers to this node's children
  // Vector< Node> children;
  // ! \brief The state or configuration associated with this node
  ArrayList<Double> state;
  // ! \brief The duration of a trajectory from the root of the search tree to the
  // state of this node
  double time = 0;
  // ! \brief The cost to reach this node from the root
  double cost = 0;
  // ! \brief The cost together with the estimated cost-to-go which is used for an
  // informed search
  double merit = 0;
  // ! \brief The index from the discrete control set used to reach this node from
  // the parent in the current instance of the algorithm
  int u_idx = 0;
  // ! \brief The depth of this node in the search tree
  int depth = 0;
  // ! \brief A flag to indicate if this Node is in the goal set
  boolean in_goal = false;

  /**
   * \brief Constructor for the Node object which sets several of the struct's
   * attributes
   * \param[in][in] card_omega_ is the cardinality of the discrete set of controls
   * used in this instance of the algorithm
   * \param[in] control_index_ is the index of the control used to reach this
   * node's state from the its parent
   * \param[in] cost_ is the cost to reach this node from the root
   * \param[in] cost_to_go_ is an underestimate of the remaining cost-to-go from
   * the state of this node
   * \param[in] state_ is a the state of the system associated with this Node
   * \param[in] time_ is the duration of the trajectory from the root to this node
   * in the search tree
   * \param[in] parent_ is a pointer to the parent of this Node
   */

  GlcNode(int _card_omega,
      int _control_index,
      double _cost,
      double _cost_to_go,
      final ArrayList<Double> _state,
      double _time,
      final GlcNode _parent,
      final InterpolatingPolynomial _trajectory_from_parent,
      final InterpolatingPolynomial _control_from_parent) {
    // children = _card_omega),
    cost = _cost;
    merit = _cost_to_go + _cost;
    time = _time;
    parent = _parent;
    state = _state;
    u_idx = _control_index;
    control_from_parent = _control_from_parent;
    trajectory_from_parent = _trajectory_from_parent;
    if (parent != null) {
      if (parent.state.get(0) != trajectory_from_parent.at(parent.time).get(0))
        throw new IllegalArgumentException();
      if (parent.state.get(1) != trajectory_from_parent.at(parent.time).get(1))
        throw new IllegalArgumentException();
    }
  }

}
