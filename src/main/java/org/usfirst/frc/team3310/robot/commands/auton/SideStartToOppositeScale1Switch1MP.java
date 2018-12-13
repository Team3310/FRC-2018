package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.commands.DriveAbsoluteTurnMP;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.DriveSpeedShift;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionPID;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;
import org.usfirst.frc.team3310.robot.commands.IntakeCubeAndLiftAbortDrive;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeedTimed;
import org.usfirst.frc.team3310.robot.commands.ParallelDelay;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveSpeedShiftState;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Intake;
import org.usfirst.frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 *
 */
public class SideStartToOppositeScale1Switch1MP extends CommandGroup {

    public SideStartToOppositeScale1Switch1MP(PathContainer path, PathContainer path2, boolean isRight) {
        
    	// Initialize everything at starting position
        addSequential(new DriveSpeedShift(DriveSpeedShiftState.HI));
    	addSequential(new ElevatorSetZero(0));
        addSequential(new DriveResetPoseFromPath(path, true));
    	addParallel(new ElevatorSetPositionMP(3.0));

    	// Drive backwards to scale.  Start raising elevator during the path when "raiseElevator" marker is crossed
    	addSequential(new DriveStraightMP(-220.0, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, true, 0));
    	addSequential(new DriveAbsoluteTurnMP(isRight ? -90 : 90, Drive.MAX_TURN_RATE_DEG_PER_SEC, MPSoftwareTurnType.TANK));  	
    	addSequential(new DriveStraightMP(-175.0, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, true, isRight ? -90 : 90));
    	addParallel(new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES));
    	addSequential(new DriveAbsoluteTurnMP(isRight ? 10 : -10, Drive.MAX_TURN_RATE_DEG_PER_SEC, MPSoftwareTurnType.TANK));  	
    	addSequential(new DriveStraightMP(-40.0, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, true, isRight ? 10 : -10));
    	addSequential(new WaitForChildren());
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_FAST_SPEED, 0.8));
    	
    	// Drive forwards to switch.  Center on last cube.  
    	addParallel(new ElevatorSetPositionPID(Elevator.MIN_POSITION_INCHES));
        addParallel(new IntakeCubeAndLiftAbortDrive(true));
        addSequential(new DriveResetPoseFromPath(path2, false));
    	addSequential(new DrivePathAdaptivePursuit(path2));
    	addSequential(new WaitForChildren());
    	
    	// Eject cube
//    	PathContainer backup5 = new Backup5();
//      addSequential(new DriveResetPoseFromPath(backup5, false));
//    	addSequential(new DrivePathAdaptivePursuit(backup5));
    	addSequential(new DriveStraightMP(-10.0, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));    	
    	addSequential(new ElevatorSetPositionPID(Elevator.SWITCH_POSITION_INCHES));

//    	PathContainer forward5 = new Forward5();
//      addSequential(new DriveResetPoseFromPath(forward5, false));
//    	addSequential(new DrivePathAdaptivePursuit(forward5));
    	addSequential(new DriveStraightMP(15.0, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, true, isRight? -10 : 10));

        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 0.6));
    	addParallel(new ParallelDelay(0.5, new ElevatorSetPositionPID(Elevator.MIN_POSITION_INCHES)));
    	addSequential(new DriveStraightMP(-25.0, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));

    }
}
