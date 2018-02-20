package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Ramp.RampPull;

import edu.wpi.first.wpilibj.command.Command;

public class RampSetPullPosition extends Command
{
	private RampPull state;
	
	public RampSetPullPosition(RampPull state) {
		requires(Robot.ramp);
		this.state = state;
	}

	@Override
	protected void initialize() {
		Robot.ramp.setPullPosition(state);
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