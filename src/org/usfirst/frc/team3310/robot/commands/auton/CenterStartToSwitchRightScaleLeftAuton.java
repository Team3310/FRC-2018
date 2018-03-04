package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.CenterStartToScaleRight;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.paths.RightSwitchToLeftScale;
import org.usfirst.frc.team3310.paths.ScaleToSwitchSameSide;
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
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 *
 */
public class CenterStartToSwitchRightScaleLeftAuton extends CommandGroup {

    public CenterStartToSwitchRightScaleLeftAuton() {
    	addSequential(new ElevatorSetZero(0));
    	
    	PathContainer path = new CenterStartToScaleRight();
        addSequential(new DriveResetPoseFromPath(path));

    	addParallel(new RunAfterMarker("raiseElevator", 4.0, new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES)));
    	addParallel(new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SLOW_SPEED, 1.0));
    	addSequential(new DrivePathAdaptivePursuit(path));
    	addSequential(new WaitForChildren());
    	addSequential(new DrivePathAdaptivePursuit(new ScaleToSwitchSameSide()));
    	addSequential(new WaitForChildren());
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 0.6));
    	addParallel(new ParallelDelay(0.7, new ElevatorSetPositionPID(Elevator.ZERO_POSITION_INCHES)));
    	addSequential(new DriveStraightMP(-12.0, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));
        addParallel(new IntakeCubeAndLiftAbortDrive());
    	addSequential(new DriveStraightMP(18.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));

    	PathContainer path2 = new RightSwitchToLeftScale();
        addSequential(new DriveResetPoseFromPath(path2));

        addParallel(new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SLOW_SPEED, 1.0));
    	addParallel(new RunAfterMarker("raiseElevator", 6.0, new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES)));
    	addSequential(new DrivePathAdaptivePursuit(path2));
    	addSequential(new WaitForChildren());
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_SPEED, 1.0));
    	addSequential(new DriveStraightMP(12.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));
        addSequential(new ElevatorSetPositionMP(Elevator.ZERO_POSITION_INCHES));
     }
}
