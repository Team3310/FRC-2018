package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class StartLeftCenterRightGoStraight extends CommandGroup {

    public StartLeftCenterRightGoStraight(PathContainer startToScalePath) {
    
    	// Initialize everything at starting position
    	addSequential(new ElevatorSetZero(0));
        addSequential(new DriveResetPoseFromPath(startToScalePath, true));

    	addSequential(new DrivePathAdaptivePursuit(startToScalePath));
     }
}
