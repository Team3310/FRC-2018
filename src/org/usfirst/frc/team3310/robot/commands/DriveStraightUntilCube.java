package org.usfirst.frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveStraightUntilCube extends CommandGroup {

    public DriveStraightUntilCube() {
        addParallel(new IntakeCubeAndLiftAbortDrive());
    	addSequential(new DriveStraightMP(100, 50, true, true, 0));
    }
}
