package edu.unc.robotics.prrts.kdtree;

import java.util.ArrayList;
import java.util.List;

public class Util {

    public static <VVV> List<VVV> values(KDNode<VVV> root) {
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

    public static <V> void insert(KDModel model, KDNode<V> root, double[] config, V value) {
        // double[] min = _min;
        // double[] max = _max;
        double[] min = new double[model.dimensions()];
        double[] max = new double[model.dimensions()];
        model.getBounds(min, max);
    
        KDNode<V> newNode = new KDNode<V>(config, value);
        KDNode<V> n = root;
        int depth = 0;
    
        for (;; ++depth) {
            int axis = depth % model.dimensions();
            double mp = (min[axis] + max[axis]) / 2;
            double v = config[axis];
    
            if (v < mp) {
                // a-side
                if (n.getA() == null) {
                    if (n.setA(null, newNode)) {
                        break;
                    }
                }
                max[axis] = mp;
                n = n.getA();
            } else {
                // b-side
                if (n.getB() == null) {
                    if (n.setB(null, newNode)) {
                        break;
                    }
                }
                min[axis] = mp;
                n = n.getB();
            }
        }
    }
    
}
