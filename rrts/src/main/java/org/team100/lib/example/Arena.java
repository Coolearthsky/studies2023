package org.team100.lib.example;

import org.team100.lib.geom.Obstacle;
import org.team100.lib.index.KDModel;
import org.team100.lib.planner.RobotModel;

public interface Arena extends RobotModel, KDModel {

    Obstacle[] obstacles();
    
}
