package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveSetFinished extends Command
{	
	public DriveSetFinished() {
	}

	@Override
	protected void initialize() {
		Robot.drive.forceDoneWithPath();
		Robot.drive.setFinished(true);
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
//    	System.out.println("DriveSetFinished interrupted");			
	}
}