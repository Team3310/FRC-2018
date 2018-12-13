package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.paths.auton.LeftStartToScaleRightAPRV2;
import org.usfirst.frc.team3310.paths.auton.ScaleRightSideToSwitchRight;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DrivePathCameraTrack;
import org.usfirst.frc.team3310.robot.commands.DriveRelativeTurnMP;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.DriveSetSpeed;
import org.usfirst.frc.team3310.robot.commands.DriveSpeedShift;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;
import org.usfirst.frc.team3310.robot.commands.IntakeCubeAndLiftAbortDrive;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeedTimed;
import org.usfirst.frc.team3310.robot.commands.RunAfterMarker;
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
public class SideStartToScale1APRV2 extends CommandGroup {

    public SideStartToScale1APRV2(PathContainer path, boolean isRight) {
        addSequential(new DriveSpeedShift(DriveSpeedShiftState.LO));
        
    	// Initialize everything at starting position
    	addSequential(new ElevatorSetZero(0));
        addSequential(new DriveResetPoseFromPath(path, true));

    	// Drive backwards to scale.  Start raising elevator during the path when "raiseElevator" marker is crossed
    	addParallel(new RunAfterMarker("shiftHi", 4.0, new DriveSpeedShift(DriveSpeedShiftState.HI)));
    	addParallel(new RunAfterMarker("shiftLow", 4.0, new DriveSpeedShift(DriveSpeedShiftState.LO)));
    	addSequential(new DrivePathAdaptivePursuit(path));
//    	addParallel(new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES));
//        addSequential(new DriveSpeedShift(DriveSpeedShiftState.LO));
    	addParallel(new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES));
    	addSequential(new DriveSetSpeed(-0.5, 0.6));
    	addSequential(new WaitForChildren());
    	addSequential(new DriveStraightMP(30, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));
        addSequential(new IntakeSetSpeedTimed(0.5*Intake.INTAKE_EJECT_SPEED, 0.5));
    	addSequential(new DriveStraightMP(-20, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));
    	addParallel(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
    	addSequential(new DriveRelativeTurnMP(isRight ? -50 : 50, Drive.MAX_TURN_RATE_DEG_PER_SEC, MPSoftwareTurnType.TANK));  	

//
//    	PathContainer path2 = new ScaleRightSideToSwitchRight();
//        addSequential(new DriveResetPoseFromPath(path2, true));
//    	addSequential(new DrivePathAdaptivePursuit(path2));
//
//        addParallel(new IntakeCubeAndLiftAbortDrive(true));
//       	addSequential(new DrivePathCameraTrack(20, 5));
//    	addSequential(new DriveStraightMP(-60, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));

    	// Drive forwards to switch.  Center on last cube.  
//        PathContainer path2 = new ScaleRightToSwitchRight();
//    	addParallel(new ElevatorSetPositionPID(Elevator.MIN_POSITION_INCHES));
//        addParallel(new IntakeCubeAndLiftAbortDrive(true));
//        addSequential(new DriveResetPoseFromPath(path2, false));
//    	addSequential(new DrivePathAdaptivePursuit(path2));
//    	addSequential(new WaitForChildren());
    	
    	// Eject cube
//    	PathContainer backup5 = new Backup5();
//        addSequential(new DriveResetPoseFromPath(backup5, false));
//    	addSequential(new DrivePathAdaptivePursuit(backup5));
    	
 //   	addSequential(new ElevatorSetPositionPID(Elevator.SWITCH_POSITION_INCHES));

//    	PathContainer forward5 = new Forward5();
//        addSequential(new DriveResetPoseFromPath(forward5, false));
//    	addSequential(new DrivePathAdaptivePursuit(forward5));

//        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 0.6));
//    	addParallel(new ParallelDelay(0.5, new ElevatorSetPositionPID(Elevator.MIN_POSITION_INCHES)));
//    	addSequential(new DriveStraightMP(-25.0, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));

    }
}
