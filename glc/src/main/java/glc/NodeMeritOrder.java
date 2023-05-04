package glc;

import java.util.Comparator;

/**
   * \brief A structure for defining a weak linear ordering of nodes based on the ordering of their merit attribute.
   *
   * This ordering is used to order a priority queue used in the A* like search. 
   */
  public class NodeMeritOrder implements Comparator<GlcNode>{

    /**
     * \brief The function defining the "less than" relation between two nodes
     * \param[in] node1 is the left element to be checked for membership in the relation
     * \param[in] node2 is the right element to be checked for membership in the relation
     * \returns true if the merit of node1 is less than the merit of node2 and false otherwise.
     */ 
    // boolean operator()(final  Node node1, final  Node node2);

    // bool NodeMeritOrder::operator()(final  GlcNode node1, final  GlcNode node2){
    //     return node1->merit>node2->merit;//negation so top of queue is min not max     
    //   }
  

  // TODO check the polarity here.

    @Override
    public int compare(GlcNode arg0, GlcNode arg1) {
      return Double.compare(arg0.merit, arg1.merit);
    }
  }