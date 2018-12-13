package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveForcePathDone extends Command
{	
	public DriveForcePathDone() {
	}

	@Override
	protected void initialize() {
		Robot.drive.forceDoneWithPath();
		Robot.drive.setFinished(true);
//		System.out.println("DRIVE FORCE PATH DONE!!!!!");
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
//   	System.out.println("DriveForcePathDone interrupted");			
	}
}