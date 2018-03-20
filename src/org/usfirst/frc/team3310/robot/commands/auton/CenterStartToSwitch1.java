package org.usfirst.frc.team3310.robot.commands.auton;

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
public class CenterStartToSwitch1 extends CommandGroup {

    public CenterStartToSwitch1(PathContainer centerStartToSwitch, boolean resetGyro) {
    	addSequential(new ElevatorSetZero(0));
    	
        addSequential(new DriveResetPoseFromPath(centerStartToSwitch, resetGyro));

        addParallel(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));
    	addSequential(new DrivePathAdaptivePursuit(centerStartToSwitch));
    	addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 0.5));
     }
}
