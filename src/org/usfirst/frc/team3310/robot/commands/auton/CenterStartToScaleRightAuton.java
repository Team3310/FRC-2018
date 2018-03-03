package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.CenterStartToScaleRight;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.paths.ScaleToSwitchSameSide;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeed;
import org.usfirst.frc.team3310.robot.commands.RunAfterMarker;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class CenterStartToScaleRightAuton extends CommandGroup {

    public CenterStartToScaleRightAuton() {
    	addSequential(new ElevatorSetZero(0));
    	
    	PathContainer path = new CenterStartToScaleRight();
        addSequential(new DriveResetPoseFromPath(path));

//        addParallel(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));
//    	addParallel(new RunAfterMarker("launchCube", 3.0, new FlipperFlip(FlipperSide.RIGHT, FlipperState.DEPLOYED)));
    	addParallel(new RunAfterMarker("raiseElevator", 4.0, new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES)));
    	addSequential(new DrivePathAdaptivePursuit(path));
        addSequential(new IntakeSetSpeed(Intake.INTAKE_REAR_EJECT_SPEED));
        addSequential(new WaitCommand(0.1));
        addSequential(new IntakeSetSpeed(0));
        addParallel(new ElevatorSetPositionMP(Elevator.ZERO_POSITION_INCHES));
    	addSequential(new DrivePathAdaptivePursuit(new ScaleToSwitchSameSide()));
     }
}
