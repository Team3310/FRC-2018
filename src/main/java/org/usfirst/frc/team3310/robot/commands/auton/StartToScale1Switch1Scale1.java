package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.paths.auton.Backup5;
import org.usfirst.frc.team3310.paths.auton.Backup5Right;
import org.usfirst.frc.team3310.paths.auton.Forward5;
import org.usfirst.frc.team3310.paths.auton.Forward5Right;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.DriveSpeedShift;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionPID;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;
import org.usfirst.frc.team3310.robot.commands.IntakeCubeAndLiftAbortDrive;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeed;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeedTimed;
import org.usfirst.frc.team3310.robot.commands.ParallelDelay;
import org.usfirst.frc.team3310.robot.commands.RunAfterMarker;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveSpeedShiftState;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 *
 */
public class StartToScale1Switch1Scale1 extends CommandGroup {

    public StartToScale1Switch1Scale1(
    		PathContainer startToScalePath, 
    		PathContainer scaleToSwitchPath1, 
    		PathContainer forwardTo2ndCube, 
    		PathContainer switchToScale2,
    		boolean isRight) {
    	
        addSequential(new IntakeSetSpeed(0.05));
    	addSequential(new DriveSpeedShift(DriveSpeedShiftState.LO));
    	addSequential(new ElevatorSetZero(0));
        addSequential(new DriveResetPoseFromPath(startToScalePath, true));

    	addParallel(new RunAfterMarker("shiftHi", 4.0, new DriveSpeedShift(DriveSpeedShiftState.HI)));
    	addParallel(new RunAfterMarker("raiseElevator", 4.0, new ElevatorSetPositionPID(Elevator.SCALE_FIRST_CUBE_POSITION_INCHES)));
    	addParallel(new RunAfterMarker("startEject", 4.0, new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_MEDIUM_SPEED, 0.8)));
    	addSequential(new DrivePathAdaptivePursuit(startToScalePath));
    	addSequential(new WaitForChildren());
    	    	
    	// Drive forward to switch platform to pickup cube  	
    	addSequential(new DriveSpeedShift(DriveSpeedShiftState.LO));
        addParallel(new ElevatorSetPositionPID(Elevator.MIN_POSITION_INCHES));
        addSequential(new DriveResetPoseFromPath(scaleToSwitchPath1, false));
        addParallel(new IntakeCubeAndLiftAbortDrive(false));
    	addSequential(new DrivePathAdaptivePursuit(scaleToSwitchPath1));
    	addSequential(new WaitForChildren());
    	
        // Back up a little, raise elevator, drive forward, then eject intake
//    	addSequential(new DriveStraightMP(-5.0, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));
    	PathContainer backup5 = isRight ? new Backup5Right() : new Backup5();
        addSequential(new DriveResetPoseFromPath(backup5, false));
    	addSequential(new DrivePathAdaptivePursuit(backup5));
    	
    	addSequential(new ElevatorSetPositionPID(Elevator.SWITCH_POSITION_INCHES));

    	PathContainer forward5 = isRight ? new Forward5Right() : new Forward5();
        addSequential(new DriveResetPoseFromPath(forward5, false));
    	addSequential(new DrivePathAdaptivePursuit(forward5));

        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 0.6));

    	// Drive backwards to scale platform that we need to eject cube  	
    	addParallel(new ParallelDelay(0.5, new ElevatorSetPositionPID(Elevator.MIN_POSITION_INCHES)));
    	addSequential(new DriveStraightMP(-25.0, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));

        addParallel(new IntakeCubeAndLiftAbortDrive(false));
        addSequential(new DriveResetPoseFromPath(forwardTo2ndCube, false));
    	addSequential(new DrivePathAdaptivePursuit(forwardTo2ndCube));
    	addSequential(new WaitForChildren());

    	// Drive backwards to scale platform that we need to eject cube  	
        addSequential(new DriveResetPoseFromPath(switchToScale2, false));
    	addParallel(new RunAfterMarker("raiseElevator", 6.0, new ElevatorSetPositionPID(Elevator.SCALE_HIGH_POSITION_INCHES)));
    	addSequential(new DrivePathAdaptivePursuit(switchToScale2));
    	addSequential(new WaitForChildren());
    	
    	// Eject cube
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_MEDIUM_SPEED, 1.0));
        addSequential(new ElevatorSetPositionPID(Elevator.ZERO_POSITION_INCHES));
    }
}
