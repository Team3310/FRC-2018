package org.usfirst.frc.team3310.paths.auton;

import java.util.ArrayList;

import org.usfirst.frc.team3310.paths.PathBuilder;
import org.usfirst.frc.team3310.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.utility.control.Path;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Rotation2d;
import org.usfirst.frc.team3310.utility.math.Translation2d;


public class LeftStartToScaleRight implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(18,274,0,0));
        sWaypoints.add(new Waypoint(220,274,40,60));
        sWaypoints.add(new Waypoint(214,210,0,60,    "raiseElevator"));
        sWaypoints.add(new Waypoint(204,150,0,60));
        sWaypoints.add(new Waypoint(204,92,30,60));
        sWaypoints.add(new Waypoint(275,87,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(18, 274), Rotation2d.fromDegrees(180)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
}