package edu.unc.robotics.prrts.example.swingup;

import org.team100.lib.index.KDModel;
import org.team100.lib.planner.RobotModel;

import edu.unc.robotics.prrts.example.geom.Obstacle;

public interface Arena extends RobotModel, KDModel {

    Obstacle[] obstacles();
    
}
