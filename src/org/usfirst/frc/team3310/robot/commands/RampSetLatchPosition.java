package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveSpeedShiftState;
import org.usfirst.frc.team3310.robot.subsystems.Ramp.RampLatch;

import edu.wpi.first.wpilibj.command.Command;

public class RampSetLatchPosition extends Command
{
	private RampLatch state;
	
	public RampSetLatchPosition(RampLatch state) {
		requires(Robot.ramp);
		this.state = state;
	}

	@Override
	protected void initialize() {
		if (Robot.ramp.getRemainingTeleopSeconds() < 30) {
			Robot.ramp.setLatchPosition(state);
		}
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