package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.DriveSpeedShift;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionPID;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;
import org.usfirst.frc.team3310.robot.commands.IntakeCubeAndLiftAbortDrive;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeedTimed;
import org.usfirst.frc.team3310.robot.commands.ParallelDelay;
import org.usfirst.frc.team3310.robot.commands.RunAfterMarker;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveSpeedShiftState;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 *
 */
public class SideStartToOppositeScale2 extends CommandGroup {

	public SideStartToOppositeScale2(PathContainer path, PathContainer path2, PathContainer switchToScale1,
			boolean isRight) {

		// Initialize everything at starting position
		addSequential(new DriveSpeedShift(DriveSpeedShiftState.LO));
		addSequential(new ElevatorSetZero(0));
        addSequential(new DriveResetPoseFromPath(path, true));

    	// Drive backwards to scale.  Start raising elevator during the path when "raiseElevator" marker is crossed
    	addParallel(new RunAfterMarker("raiseElevator", 8.0, new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES)));
    	addSequential(new DrivePathAdaptivePursuit(path));
    	addSequential(new WaitForChildren());
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_FAST_SPEED, 0.8));
    	
    	// Drive forwards to switch.  Center on last cube.  
    	addParallel(new ElevatorSetPositionPID(Elevator.MIN_POSITION_INCHES));
        addParallel(new IntakeCubeAndLiftAbortDrive(true));
        addSequential(new DriveResetPoseFromPath(path2, false));
    	addSequential(new DrivePathAdaptivePursuit(path2));
    	addSequential(new WaitForChildren());
    	
    	// Drive backwards to scale platform that we need to eject cube  	
//    	addParallel(new ElevatorSetPositionPID(Elevator.SCALE_HIGH_POSITION_INCHES));
        addSequential(new DriveResetPoseFromPath(switchToScale1, false));
        addParallel(new ParallelDelay(0.3, new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SLOW_SPEED, 0.2)));
        addParallel(new RunAfterMarker("raiseElevator", 6.0, new ElevatorSetPositionPID(Elevator.SCALE_HIGH_POSITION_INCHES)));
//   	addParallel(new RunAfterMarker("startEject", 4.0, new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_SPEED, 1.0)));
    	addSequential(new DrivePathAdaptivePursuit(switchToScale1));
    	addSequential(new WaitForChildren());
    	    	
    	// Eject cube
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_MEDIUM_SPEED, 0.8));

    	// Drive forward to switch platform to pickup cube  	
        addParallel(new ElevatorSetPositionPID(Elevator.ZERO_POSITION_INCHES));
    }
}
