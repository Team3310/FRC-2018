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
import org.usfirst.frc.team3310.robot.commands.RunAfterMarker;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveSpeedShiftState;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 *
 */
public class StartToScale3 extends CommandGroup {

    public StartToScale3(
    		PathContainer startToScalePath, 
    		PathContainer scaleToSwitchPath1, 
    		PathContainer switchToScale1, 
    		PathContainer scaleToSwitchPath2, 
    		PathContainer switchToScale2) {
    	
    	addSequential(new DriveSpeedShift(DriveSpeedShiftState.HI));
    	addSequential(new ElevatorSetZero(0));
        addSequential(new DriveResetPoseFromPath(startToScalePath, true));

    	addParallel(new RunAfterMarker("raiseElevator", 4.0, new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES)));
    	addParallel(new RunAfterMarker("startEject", 4.0, new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_SPEED, 1.0)));
    	addSequential(new DrivePathAdaptivePursuit(startToScalePath));
    	addSequential(new WaitForChildren());

    	// Drive forward to switch platform to pickup cube  	
//        addParallel(new ElevatorSetPositionMP(Elevator.ZERO_POSITION_INCHES));
    	addSequential(new DriveSpeedShift(DriveSpeedShiftState.LO));
        addParallel(new ElevatorSetPositionPID(Elevator.ZERO_POSITION_INCHES));
        addSequential(new WaitCommand(0.3));
        addParallel(new IntakeCubeAndLiftAbortDrive());
        addSequential(new DriveResetPoseFromPath(scaleToSwitchPath1, false));
    	addSequential(new DrivePathAdaptivePursuit(scaleToSwitchPath1));
    	addSequential(new WaitForChildren());

    	// Drive backwards to scale platform that we need to eject cube  	
    	addParallel(new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES));
        addSequential(new DriveResetPoseFromPath(switchToScale1, false));
//    	addParallel(new RunAfterMarker("raiseElevator", 6.0, new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES)));
//   	addParallel(new RunAfterMarker("startEject", 4.0, new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_SPEED, 1.0)));
    	addSequential(new DrivePathAdaptivePursuit(switchToScale1));
    	addSequential(new WaitForChildren());
    	
    	// Eject cube
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_SPEED, 1.0));

    	// Drive forward to switch platform to pickup cube  	
        addParallel(new ElevatorSetPositionMP(Elevator.ZERO_POSITION_INCHES));
        addSequential(new WaitCommand(0.8));
        addParallel(new IntakeCubeAndLiftAbortDrive());
        addSequential(new DriveResetPoseFromPath(scaleToSwitchPath2, false));
    	addSequential(new DrivePathAdaptivePursuit(scaleToSwitchPath2));
    	addSequential(new WaitForChildren());

    	// Drive backwards to scale platform that we need to eject cube  	
        addSequential(new DriveResetPoseFromPath(switchToScale2, false));
    	addParallel(new RunAfterMarker("raiseElevator", 6.0, new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES)));
    	addSequential(new DrivePathAdaptivePursuit(switchToScale2));
    	addSequential(new WaitForChildren());
    	
    	// Eject cube
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_SPEED, 1.0));
    }
}
