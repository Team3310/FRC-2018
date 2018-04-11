package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionPID;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;
import org.usfirst.frc.team3310.robot.commands.IntakeCubeAndLiftAbortDrive;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeedTimed;
import org.usfirst.frc.team3310.robot.commands.ParallelDelay;
import org.usfirst.frc.team3310.robot.commands.RunAfterMarker;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 *
 */
public class SideStartToSwitch1PickupLastCube extends CommandGroup {

    public SideStartToSwitch1PickupLastCube(PathContainer startToScalePath, PathContainer scaleToSwitchPath) {
    
    	// Initialize everything at starting position
    	addSequential(new ElevatorSetZero(0));
        addSequential(new DriveResetPoseFromPath(startToScalePath, true));

    	// Drive backwards to scale.  Start raising elevator during the path when "raiseElevator" marker is crossed
    	addParallel(new RunAfterMarker("raiseElevator", 4.0, new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES)));
//    	addParallel(new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SLOW_SPEED, 1.0));
    	addSequential(new DrivePathAdaptivePursuit(startToScalePath));
    	addSequential(new WaitForChildren());
    	
    	// Drive forwards to switch.  Center on last cube.  
        addSequential(new DriveResetPoseFromPath(scaleToSwitchPath, false));
    	addSequential(new DrivePathAdaptivePursuit(scaleToSwitchPath));
    	
    	// Eject cube
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 0.6));
        
        // Back up, lower elevator, start intake, wait until cube acquired
    	addParallel(new ParallelDelay(0.7, new ElevatorSetPositionPID(Elevator.ZERO_POSITION_INCHES)));
    	addSequential(new DriveStraightMP(-12.0, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));
        addParallel(new IntakeCubeAndLiftAbortDrive(false));
    	addSequential(new DriveStraightMP(22.0, Drive.MP_SLOW_VELOCITY_INCHES_PER_SEC, true, false, 0));
    	addParallel(new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SLOW_SPEED, 0.2));
     }
}
