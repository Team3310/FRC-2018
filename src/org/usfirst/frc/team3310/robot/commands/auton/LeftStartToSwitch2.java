package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.commands.DriveAbsoluteTurnMP;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;
import org.usfirst.frc.team3310.robot.commands.IntakeCubeAndLiftAbortDrive;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeedTimed;
import org.usfirst.frc.team3310.robot.commands.RunAfterMarker;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Intake;
import org.usfirst.frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftStartToSwitch2 extends CommandGroup {

    public LeftStartToSwitch2(PathContainer leftStartToCenterStart, PathContainer switchToCenter, PathContainer centerToSwitch) {
    	addSequential(new ElevatorSetZero(0));
    	addSequential(new DriveStraightMP(-60, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, true, 0));
    	addSequential(new DriveAbsoluteTurnMP(-90, Drive.MP_AUTON_MAX_TURN_RATE_DEG_PER_SEC, MPSoftwareTurnType.TANK));
    	
        addSequential(new DriveResetPoseFromPath(leftStartToCenterStart, true));
    	addParallel(new RunAfterMarker("raiseElevator", 4.0, new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES)));
    	addSequential(new DrivePathAdaptivePursuit(leftStartToCenterStart));
		addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 0.5));

		addSequential(new DriveResetPoseFromPath(switchToCenter, true));
		addParallel(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
		addSequential(new DrivePathAdaptivePursuit(switchToCenter));

		addParallel(new IntakeCubeAndLiftAbortDrive(false));
		addSequential(new DriveStraightMP(50, Drive.MP_SLOW_VELOCITY_INCHES_PER_SEC, true, true, 0));

		addSequential(new DriveStraightMP(-40, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, true, 0));
		addSequential(new CenterStartToSwitch1(centerToSwitch, false, 0.0));

		addParallel(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
		addSequential(new DriveStraightMP(-15, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, true, 0));
	}
}
