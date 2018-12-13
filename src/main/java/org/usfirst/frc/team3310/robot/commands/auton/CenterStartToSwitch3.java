package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.IntakeCubeAndLiftAbortDrive;
import org.usfirst.frc.team3310.robot.commands.ParallelDelay;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CenterStartToSwitch3 extends CommandGroup {

    public CenterStartToSwitch3(PathContainer centerStartToSwitch, PathContainer switchToCenter, PathContainer centerToSwitch, PathContainer switchToNullZone) {
    	addSequential(new CenterStartToSwitch1(centerStartToSwitch, true, 0.75));
    	
        addSequential(new DriveResetPoseFromPath(switchToCenter, false));
        addParallel(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
    	addSequential(new DrivePathAdaptivePursuit(switchToCenter));
    	
        addParallel(new IntakeCubeAndLiftAbortDrive(false));
    	addSequential(new DriveStraightMP(32, Drive.MP_SLOW_MEDIUM_VELOCITY_INCHES_PER_SEC, true, true, 0));

    	addSequential(new DriveStraightMP(-40, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, true, 0));
    	addSequential(new CenterStartToSwitch1(centerToSwitch, false, 0.0));
    	
        addParallel(new ParallelDelay(0.5, new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES)));
    	addSequential(new DriveStraightMP(-50, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, true, 0));

        addSequential(new DriveResetPoseFromPath(switchToNullZone, false));
        addSequential(new DrivePathAdaptivePursuit(switchToNullZone));
        }
}
