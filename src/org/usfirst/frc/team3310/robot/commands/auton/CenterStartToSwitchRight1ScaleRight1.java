package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.CenterStartToScaleRight;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.paths.RightSwitchToLeftScale;
import org.usfirst.frc.team3310.paths.ScaleToSwitchSameSideRight;
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
public class CenterStartToSwitchRight1ScaleRight1 extends CommandGroup {

    public CenterStartToSwitchRight1ScaleRight1() {
    
    	// Drive backwards to scale, drive forward, eject cube, pickup last cube
    	addSequential(new CenterStartToSwitch1PickupLastCube(new CenterStartToScaleRight(), new ScaleToSwitchSameSideRight()));

    	// Would like to get rid of this...
    	PathContainer path2 = new RightSwitchToLeftScale();
        addSequential(new DriveResetPoseFromPath(path2));

    	// Drive to opposite scale  	
        addParallel(new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SLOW_SPEED, 1.0));
    	addParallel(new RunAfterMarker("raiseElevator", 6.0, new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES)));
    	addSequential(new DrivePathAdaptivePursuit(path2));
    	addSequential(new WaitForChildren());
    	
    	// Eject cube
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_SPEED, 1.0));
    	addSequential(new DriveStraightMP(12.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));
        addSequential(new ElevatorSetPositionMP(Elevator.ZERO_POSITION_INCHES));
     }
}
