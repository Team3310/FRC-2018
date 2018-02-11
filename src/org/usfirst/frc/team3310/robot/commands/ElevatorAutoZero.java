package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorAutoZero extends Command
{
	public ElevatorAutoZero() {
		requires(Robot.elevator);
	}

	@Override
	protected void initialize() {
		Robot.elevator.setSpeed(-Elevator.TEST_SPEED);
	}

	@Override
	protected void execute() {
		
	}

	@Override
	protected boolean isFinished() {
		return Robot.elevator.getMotorCurrent() > Elevator.AUTO_ZERO_MOTOR_CURRENT;
	}

	@Override
	protected void end() {
		Robot.elevator.resetZeroPosition();
	}

	@Override
	protected void interrupted() {
			
	}
}