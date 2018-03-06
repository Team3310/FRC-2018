package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.CenterStartToScaleRight;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.paths.ScaleRightToSwitchRight;
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

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 *
 */
public class CenterStartToScaleRight1SwitchRight1 extends CommandGroup {

    public CenterStartToScaleRight1SwitchRight1() {
    	addSequential(new ElevatorSetZero(0));
    	
    	PathContainer path = new CenterStartToScaleRight();
        addSequential(new DriveResetPoseFromPath(path, true));

    	addParallel(new RunAfterMarker("raiseElevator", 4.0, new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES)));
    	addSequential(new DrivePathAdaptivePursuit(path));
    	addSequential(new WaitForChildren());
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_SPEED, 1.0));

        addParallel(new ElevatorSetPositionMP(Elevator.ZERO_POSITION_INCHES));
        addSequential(new WaitCommand(0.8));
        addParallel(new IntakeCubeAndLiftAbortDrive());
    	addSequential(new DrivePathAdaptivePursuit(new ScaleRightToSwitchRight()));
    	addSequential(new WaitForChildren());

    	addSequential(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));
    	addSequential(new DriveStraightMP(12.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 1.0));
     }
}
