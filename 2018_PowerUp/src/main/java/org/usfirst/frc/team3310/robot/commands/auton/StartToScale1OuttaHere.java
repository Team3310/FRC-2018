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
public class StartToScale1OuttaHere extends CommandGroup {

    public StartToScale1OuttaHere(
    		PathContainer startToScalePath, 
    		PathContainer scaleToOuttaHere
    	) {
    	
    	addSequential(new DriveSpeedShift(DriveSpeedShiftState.LO));
    	addSequential(new ElevatorSetZero(0));
        addSequential(new DriveResetPoseFromPath(startToScalePath, true));

    	addParallel(new RunAfterMarker("shiftHi", 4.0, new DriveSpeedShift(DriveSpeedShiftState.HI)));
    	addParallel(new RunAfterMarker("raiseElevator", 4.0, new ElevatorSetPositionPID(Elevator.SCALE_FIRST_CUBE_POSITION_INCHES)));
    	addParallel(new RunAfterMarker("startEject", 4.0, new IntakeSetSpeedTimed(1.0, 1.0)));
    	addSequential(new DrivePathAdaptivePursuit(startToScalePath));
    	addSequential(new DriveSpeedShift(DriveSpeedShiftState.LO));
//    	addSequential(new WaitForChildren());
    	    	
    	// Drive forward to switch platform to pickup cube  	
        addParallel(new ElevatorSetPositionPID(Elevator.MIN_POSITION_INCHES));
        addSequential(new DriveResetPoseFromPath(scaleToOuttaHere, false));
    	addSequential(new DrivePathAdaptivePursuit(scaleToOuttaHere));
    	addSequential(new WaitForChildren());
    }
}
