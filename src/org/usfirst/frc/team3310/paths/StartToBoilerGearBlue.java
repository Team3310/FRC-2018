package org.usfirst.frc.team3310.paths;

import org.usfirst.frc.team3310.paths.profiles.PathAdapter;
import org.usfirst.frc.team3310.utility.control.Path;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;

/**
 * Path from the blue alliance wall to the blue boiler peg.
 * 
 * Used in GearThenHopperShootModeBlue
 * 
 * @see GearThenHopperShootModeBlue
 * @see PathContainer
 */
public class StartToBoilerGearBlue implements PathContainer {

    @Override
    public Path buildPath() {
        return PathAdapter.getBlueGearPath();
    }

    @Override
    public RigidTransform2d getStartPose() {
        return PathAdapter.getBlueStartPose();
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}