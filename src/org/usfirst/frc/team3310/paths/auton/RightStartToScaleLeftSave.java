package org.usfirst.frc.team3310.paths.auton;

import java.util.ArrayList;

import org.usfirst.frc.team3310.paths.PathBuilder;
import org.usfirst.frc.team3310.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.utility.control.Path;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Rotation2d;
import org.usfirst.frc.team3310.utility.math.Translation2d;


public class RightStartToScaleLeftSave implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(18,51,0,110));
        sWaypoints.add(new Waypoint(159,51,0,110));
        sWaypoints.add(new Waypoint(220,51,40,110));
        sWaypoints.add(new Waypoint(214,115,0,60));
        sWaypoints.add(new Waypoint(210,145,0,100));
        sWaypoints.add(new Waypoint(209,175,0,80));
        sWaypoints.add(new Waypoint(209,205,0,70));
        sWaypoints.add(new Waypoint(214,238,30,50));
        sWaypoints.add(new Waypoint(250,238,0,50,   "raiseElevator"));
        sWaypoints.add(new Waypoint(270,235,0,40));

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