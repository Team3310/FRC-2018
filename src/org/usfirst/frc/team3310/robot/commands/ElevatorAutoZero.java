package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorAutoZero extends Command
{
	public ElevatorAutoZero(boolean interrutible) {
		requires(Robot.intake);
		requires(Robot.elevator);
		setInterruptible(interrutible);
	}

	@Override
	protected void initialize() {
		Robot.elevator.setSpeed(Elevator.AUTO_ZERO_SPEED);
		System.out.println("Auto zero initialize");
	}

	@Override
	protected void execute() {
		
	}

	@Override
	protected boolean isFinished() {
		Robot.elevator.setSpeed(Elevator.AUTO_ZERO_SPEED);
		return Robot.elevator.getMotorCurrent() > Elevator.AUTO_ZERO_MOTOR_CURRENT;
	}

	@Override
	protected void end() {
		Robot.elevator.setSpeed(0);
		Robot.elevator.resetZeroPosition(Elevator.ZERO_POSITION_INCHES);
		Robot.elevator.setPositionPID(Elevator.MIN_POSITION_INCHES);
		System.out.println("Elevator Zeroed");
	}

	@Override
	protected void interrupted() {
			
	}
}