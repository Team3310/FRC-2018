package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Forks.ForksLockState;

import edu.wpi.first.wpilibj.command.Command;

public class ForksSetLock extends Command
{
	private ForksLockState state;
	
	public ForksSetLock(ForksLockState state) {
		requires(Robot.forks);
		this.state = state;
	}

	@Override
	protected void initialize() {
		Robot.forks.setLockState(state);
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