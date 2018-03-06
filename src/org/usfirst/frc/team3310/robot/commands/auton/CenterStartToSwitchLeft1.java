package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.CenterStartToSwitchLeft;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeedTimed;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CenterStartToSwitchLeft1 extends CommandGroup {

    public CenterStartToSwitchLeft1() {
    	addSequential(new ElevatorSetZero(0));
    	
    	PathContainer path = new CenterStartToSwitchLeft();
        addSequential(new DriveResetPoseFromPath(path));

        addParallel(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));
    	addSequential(new DrivePathAdaptivePursuit(path));
    	addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 0.5));
     }
}
