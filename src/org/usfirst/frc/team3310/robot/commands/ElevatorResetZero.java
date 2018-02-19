package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorResetZero extends Command
{
	public ElevatorResetZero() {
		requires(Robot.elevator);
	}

	@Override
	protected void initialize() {
		Robot.elevator.resetZeroPosition(Elevator.ZERO_POSITION_INCHES);
	}

	@Override
	protected void execute() {
		
	}

	@Override
	protected boolean isFinished() {
		return true;
	}

	@Override
	protected void end() {
		
	}

	@Override
	protected void interrupted() {
			
	}
}