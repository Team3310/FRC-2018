package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Elevator.SpeedShiftState;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorSpeedShift extends Command
{
	private SpeedShiftState state;
	
	public ElevatorSpeedShift(SpeedShiftState state) {
		requires(Robot.elevator);
		this.state = state;
	}

	@Override
	protected void initialize() {
		Robot.elevator.setShiftState(state);
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