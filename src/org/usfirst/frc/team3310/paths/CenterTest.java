package org.usfirst.frc.team3310.paths;

import java.util.ArrayList;

import org.usfirst.frc.team3310.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team3310.utility.control.Path;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Rotation2d;
import org.usfirst.frc.team3310.utility.math.Translation2d;

public class CenterTest implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(0, 0, 0, 0));
        sWaypoints.add(new Waypoint(20, 0, 0, 40));
        sWaypoints.add(new Waypoint(40, 0, 0, 40));
        sWaypoints.add(new Waypoint(60, 0, 0, 60));
        sWaypoints.add(new Waypoint(80, 0, 0, 40));
        sWaypoints.add(new Waypoint(100, 0, 0, 40));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}