package glc;

import java.util.PriorityQueue;
import java.util.Vector;

/**
 * This class defines the equivalence classes (i.e. partition) of the state space
 * 
 * The GLC algorithm is able to build a sparse search tree by maintaining at most one node
 * per equivalence class. The size of the equivalence class is determined by the resolution
 * parameter in the current planning query.
 */
class GlcStateEquivalenceClass{
  
  // Each equivalence class (a hyper-cubicle region) is uniquely identified by an integer tuple
  Vector<Integer> coordinate;
  // An equivalence class has a pointer to a node whose associated state is in the cubicle region
   GlcNode label;
  /**
   * A priority queue of potential new nodes that could label the cell
   * 
   * Once relabeled, the subtree rooted at the old label is deleted.
   */ 
  PriorityQueue< GlcNode> candidates = new PriorityQueue<GlcNode>(new NodeMeritOrder());
  
  //!  The constructor is the default constructor
  public GlcStateEquivalenceClass() {
    label = new GlcNode(0, 
  -1, 
  Double.MAX_VALUE, 
Double.MAX_VALUE,
  new double[0],
  0,
  null,
  null,
  null);


}

  //! \brief If no label has been set and the label attribute is nullptr, then empty will return true -- and false otherwise
  boolean empty() {
    return label.cost == Double.MAX_VALUE;
  }

  //! \brief Lexicographical ordering is used to support std::map
  boolean operator<(final GlcStateEquivalenceClass y) {
  if (coordinate.size() != y.coordinate.size()) throw new IllegalArgumentException();
   return std::lexicographical_compare <Vector<int>::const_iterator, Vector<int>::const_iterator>
   (coordinate.begin(), coordinate.end(), y.coordinate.begin(), y.coordinate.end());
 }



}
 