package org.team100.lib.example;

import org.team100.lib.geom.Obstacle;
import org.team100.lib.index.KDModel;
import org.team100.lib.planner.RobotModel;

import edu.wpi.first.math.Num;

public interface Arena<States extends Num> extends  KDModel<States>, RobotModel<States> {

    Obstacle[] obstacles();
    
}
