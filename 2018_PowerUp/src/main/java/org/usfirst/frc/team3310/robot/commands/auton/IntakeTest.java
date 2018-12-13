package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.DriveSpeedShift;
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
public class IntakeTest extends CommandGroup {

    public IntakeTest(
    		PathContainer startToScalePath, 
    		PathContainer scaleToSwitchPath1, 
    		PathContainer switchToScale1, 
    		PathContainer scaleToSwitchPath2, 
    		PathContainer switchToScale2) {
    	
    	addSequential(new ElevatorSetZero(0));
    	
    	// Drive forward to switch platform to pickup cube  	
    	addSequential(new DriveSpeedShift(DriveSpeedShiftState.LO));
        addSequential(new DriveResetPoseFromPath(scaleToSwitchPath1, true));
        addParallel(new IntakeCubeAndLiftAbortDrive(false));
    	addSequential(new DrivePathAdaptivePursuit(scaleToSwitchPath1));

    	// Drive backwards to scale platform that we need to eject cube  	
        addSequential(new DriveResetPoseFromPath(switchToScale1, false));
//        addParallel(new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SPEED, 0.3));
        addParallel(new ParallelDelay(0.3, new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SLOW_SPEED, 0.2)));
        addParallel(new RunAfterMarker("raiseElevator", 6.0, new ElevatorSetPositionPID(Elevator.SCALE_HIGH_POSITION_INCHES)));
    	addSequential(new DrivePathAdaptivePursuit(switchToScale1));
    	addSequential(new WaitForChildren());
     }
}
