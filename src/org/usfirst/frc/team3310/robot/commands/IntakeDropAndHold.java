package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class IntakeDropAndHold extends CommandGroup {

    public IntakeDropAndHold(double speed, double timeout) {
    	addSequential(new IntakeSetSpeed(speed));
    	addSequential(new WaitCommand(timeout));
    	addSequential(new IntakeSetSpeed(0));
       	addSequential(new WaitCommand(1.0));
        addSequential(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));
    }
}