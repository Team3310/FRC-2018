package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.CenterStartToSwitchRight;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionPID;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeed;
import org.usfirst.frc.team3310.robot.commands.ParallelDelay;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class CenterStartToSwitchRightAuton extends CommandGroup {

    public CenterStartToSwitchRightAuton() {
//    	addSequential(new ElevatorSetZero(Elevator.ZERO_POSITION_AUTON_FORWARD_INCHES));
//    	addSequential(new ElevatorSetPositionMP(5));
    	addSequential(new ElevatorSetZero(0));
    	
    	PathContainer path = new CenterStartToSwitchRight();
        addSequential(new DriveResetPoseFromPath(path));

        addParallel(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));
    	addSequential(new DrivePathAdaptivePursuit(path));
    	addSequential(new IntakeSetSpeed(Intake.INTAKE_EJECT_SPEED));
    	addSequential(new WaitCommand(0.5));
    	addSequential(new IntakeSetSpeed(0));
     }
}
