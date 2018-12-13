package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeedTimed;
import org.usfirst.frc.team3310.robot.commands.RunAfterMarker;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 *
 */
public class SideStartToSwitch1Scale1 extends CommandGroup {

    public SideStartToSwitch1Scale1(PathContainer startToScale, PathContainer scaleToSwitch, PathContainer switchToScale) {
    
    	// Drive backwards to scale, drive forward, eject cube, pickup last cube
    	addSequential(new SideStartToSwitch1PickupLastCube(startToScale, scaleToSwitch));

    	// Drive backwards to scale platform that we need to eject cube  	
        addSequential(new DriveResetPoseFromPath(switchToScale, false));
    	addParallel(new RunAfterMarker("raiseElevator", 6.0, new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES)));
    	addSequential(new DrivePathAdaptivePursuit(switchToScale));
    	addSequential(new WaitForChildren());
    	
    	// Eject cube
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_FAST_SPEED, 1.0));
    	addSequential(new DriveStraightMP(12.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));
        addSequential(new ElevatorSetPositionMP(Elevator.ZERO_POSITION_INCHES));
     }
}
