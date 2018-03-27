package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.IntakeCubeAndLiftAbortDrive;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CenterStartToSwitch2 extends CommandGroup {

    public CenterStartToSwitch2(PathContainer centerStartToSwitch, PathContainer switchToCenter, PathContainer centerToSwitch) {
    	addSequential(new CenterStartToSwitch1(centerStartToSwitch, true));
    	
        addSequential(new DriveResetPoseFromPath(switchToCenter, false));
        addParallel(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
    	addSequential(new DrivePathAdaptivePursuit(switchToCenter));
    	
        addParallel(new IntakeCubeAndLiftAbortDrive(false));
    	addSequential(new DriveStraightMP(32, Drive.MP_SLOW_VELOCITY_INCHES_PER_SEC, true, true, 0));

    	addSequential(new DriveStraightMP(-40, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, true, 0));
    	addSequential(new CenterStartToSwitch1(centerToSwitch, false));
    	
        addParallel(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
    	addSequential(new DriveStraightMP(-15, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, true, 0));
    }
}
