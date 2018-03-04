package org.usfirst.frc.team3310.paths;

import java.util.ArrayList;

import org.usfirst.frc.team3310.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team3310.utility.control.Path;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Rotation2d;
import org.usfirst.frc.team3310.utility.math.Translation2d;

public class ScaleToSwitchSameSideRight implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();

        sWaypoints.add(new Waypoint(280, 105, 0,  0));
        sWaypoints.add(new Waypoint(215, 112, 0, 60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(280, 105), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":20,"y":60},"speed":0,"radius":0,"comment":""},{"position":{"x":60,"y":60},"speed":15,"radius":30,"comment":""},{"position":{"x":60,"y":100},"speed":15,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftTurnRadius
}