package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveSpeedShiftState;

import edu.wpi.first.wpilibj.command.Command;

public class DriveSpeedShift extends Command
{
	private DriveSpeedShiftState state;
	
	public DriveSpeedShift(DriveSpeedShiftState state) {
//		requires(Robot.drive);
		this.state = state;
	}

	@Override
	protected void initialize() {
		Robot.drive.setShiftState(state);
		Robot.drive.configureTalonsForSpeedControl();
		System.out.println("Shift " + state);
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