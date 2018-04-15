package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.paths.auton.RightStartToScaleLeftAPR;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveRelativeTurnMP;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.DriveSetSpeed;
import org.usfirst.frc.team3310.robot.commands.DriveSpeedShift;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeed;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeedTimed;
import org.usfirst.frc.team3310.robot.commands.ParallelDelay;
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
public class SideStartToScale1APR extends CommandGroup {

    public SideStartToScale1APR(PathContainer path, boolean isRight) {
        addSequential(new DriveSpeedShift(DriveSpeedShiftState.LO));
        addSequential(new IntakeSetSpeed(0.05));
       
    	// Initialize everything at starting position
    	addSequential(new ElevatorSetZero(0));
        addSequential(new DriveResetPoseFromPath(path, true));

    	// Drive backwards to scale.  Start raising elevator during the path when "raiseElevator" marker is crossed
    	addSequential(new DrivePathAdaptivePursuit(path));
    	addParallel(new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES));
    	addSequential(new DriveRelativeTurnMP(isRight ? -90 : 90, Drive.MAX_TURN_RATE_DEG_PER_SEC, MPSoftwareTurnType.TANK));  	
    	addSequential(new DriveSetSpeed(-0.3, 0.5));
    	addSequential(new ParallelDelay(1.0, new DriveStraightMP(30, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0)));
    	addSequential(new WaitForChildren());
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 0.5));
    	addSequential(new DriveStraightMP(-20, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));
    	addParallel(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
    	addSequential(new DriveRelativeTurnMP(isRight ? -50 : 50, Drive.MAX_TURN_RATE_DEG_PER_SEC, MPSoftwareTurnType.TANK));  	
    }
}
