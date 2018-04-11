package org.usfirst.frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveChaseCube extends CommandGroup {

    public DriveChaseCube() {
        addParallel(new IntakeCubeAndLiftAbortDrive(true));
        addSequential(new DrivePathCameraTrack(40, 10));
    }
}
