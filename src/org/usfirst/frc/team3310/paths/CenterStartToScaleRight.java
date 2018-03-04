package org.usfirst.frc.team3310.paths;

import java.util.ArrayList;

import org.usfirst.frc.team3310.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team3310.utility.control.Path;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Rotation2d;
import org.usfirst.frc.team3310.utility.math.Translation2d;

public class CenterStartToScaleRight implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(19,157, 0, 0));
        sWaypoints.add(new Waypoint(40,157,20,75));
        sWaypoints.add(new Waypoint(115,45,60,75,"raiseElevator"));
        sWaypoints.add(new Waypoint(210,75, 0,75));
        sWaypoints.add(new Waypoint(250,95,20,75));
        sWaypoints.add(new Waypoint(280,105, 0,75));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(19, 157), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":19,"y":157},"speed":0,"radius":0,"comment":""},{"position":{"x":40,"y":157},"speed":60,"radius":20,"comment":""},{"position":{"x":100,"y":105},"speed":60,"radius":20,"comment":""},{"position":{"x":122,"y":105},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: CenterStartToSwitchRight
}