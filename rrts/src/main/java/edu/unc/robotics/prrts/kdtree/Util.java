package edu.unc.robotics.prrts.kdtree;

import java.util.ArrayList;
import java.util.List;

public class Util {

    public static <VVV> Iterable<VVV> values(KDNode<VVV> root) {
        List<VVV> list = new ArrayList<VVV>();
        buildList(list, root);
        return list;
    }

	static <VV> void buildList(List<VV> list, KDNode<VV> node) {
	    if (node == null)
	        return;
	
	    list.add(node.getValue());
	    buildList(list, node.getA());
	    buildList(list, node.getB());
	}
    
}
