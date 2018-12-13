package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.paths.auton.Backup5;
import org.usfirst.frc.team3310.paths.auton.Backup5Right;
import org.usfirst.frc.team3310.paths.auton.Forward5;
import org.usfirst.frc.team3310.paths.auton.Forward5Right;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveRelativeTurnMP;
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
import org.usfirst.frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 *
 */
public class SideStartToSwitch3 extends CommandGroup {

    public SideStartToSwitch3(PathContainer startToScale, PathContainer scaleToSwitch, PathContainer forwardTo2ndCube, boolean isRight) {
    
    	// Drive backwards to scale, drive forward, eject cube, pickup last cube
    	// Initialize everything at starting position
    	addSequential(new ElevatorSetZero(0));
        addSequential(new DriveResetPoseFromPath(startToScale, true));

    	// Drive backwards to scale.  Start raising elevator during the path when "raiseElevator" marker is crossed
    	addParallel(new RunAfterMarker("raiseElevator", 4.0, new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_HIGH_INCHES)));
//    	addParallel(new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SLOW_SPEED, 1.0));
    	addSequential(new DrivePathAdaptivePursuit(startToScale));
    	addSequential(new WaitForChildren());
    	
    	// Drive forwards to switch.  Center on last cube.  
        addSequential(new DriveResetPoseFromPath(scaleToSwitch, false));
    	addSequential(new DrivePathAdaptivePursuit(scaleToSwitch));
    	
    	// Eject cube
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_FAST_SPEED, 0.6));
        
        // Back up, lower elevator, start intake, wait until cube acquired
    	addParallel(new ParallelDelay(0.7, new ElevatorSetPositionPID(Elevator.ZERO_POSITION_INCHES)));
    	addSequential(new DriveStraightMP(-12.0, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));
        addParallel(new IntakeCubeAndLiftAbortDrive(false));
    	addSequential(new DriveStraightMP(25.0, Drive.MP_SLOW_VELOCITY_INCHES_PER_SEC, true, false, 0));

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
    	addParallel(new ElevatorSetPositionPID(Elevator.SWITCH_POSITION_INCHES));
    	addSequential(new DriveRelativeTurnMP(isRight ? -50 : 50, Drive.MAX_TURN_RATE_DEG_PER_SEC, MPSoftwareTurnType.TANK));  	
//        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 1.0));

//    	// Drive backwards to scale platform that we need to eject cube  	
//    	PathContainer backup6 = isRight ? new Backup5Right() : new Backup5();
//        addSequential(new DriveResetPoseFromPath(backup6, false));
//    	addParallel(new RunAfterMarker("raiseElevator", 6.0, new ElevatorSetPositionPID(Elevator.SWITCH_POSITION_INCHES)));
//    	addSequential(new DrivePathAdaptivePursuit(backup6));
//    	addSequential(new WaitForChildren());
//    	addSequential(new DriveStraightMP(25.0, Drive.MP_SLOW_VELOCITY_INCHES_PER_SEC, true, false, 0));
//    	
//    	// Eject cube
//        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 1.0));
     }
}
