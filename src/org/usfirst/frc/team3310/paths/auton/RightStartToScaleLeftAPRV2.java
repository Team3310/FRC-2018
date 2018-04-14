package org.usfirst.frc.team3310.paths.auton;

import java.util.ArrayList;

import org.usfirst.frc.team3310.paths.PathBuilder;
import org.usfirst.frc.team3310.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.utility.control.Path;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Rotation2d;
import org.usfirst.frc.team3310.utility.math.Translation2d;


public class RightStartToScaleLeftAPRV2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(18,51,0,40));
        sWaypoints.add(new Waypoint(55,51,20,40));
        sWaypoints.add(new Waypoint(55,75,0,110,     "shiftHi"));
        sWaypoints.add(new Waypoint(55,120,0,110));
        sWaypoints.add(new Waypoint(55,215,80,110));
        sWaypoints.add(new Waypoint(140,210,0,80));
        sWaypoints.add(new Waypoint(270,210,0,80,   "shiftLow"));
        sWaypoints.add(new Waypoint(300,205,0,80,  "raiseElevator"));
        sWaypoints.add(new Waypoint(340,205,20,80));
        sWaypoints.add(new Waypoint(340,225,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(18, 51), Rotation2d.fromDegrees(180)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
}