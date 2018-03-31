package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc.team3310.utility.control.Path;

import edu.wpi.first.wpilibj.command.Command;

public class DrivePathAdaptivePursuit extends Command
{
	private Path path;
	private PathContainer pathContainer;

	public DrivePathAdaptivePursuit(PathContainer p) {
        this.pathContainer = p;
        this.path = this.pathContainer.buildPath();
		requires(Robot.drive);
	}

	protected void initialize() {
		Robot.drive.setWantDrivePath(path, pathContainer.isReversed());
//		System.out.println("Adaptive Pursuit start");
	}

	protected void execute() {
	}

	protected boolean isFinished() {
		return Robot.drive.isDoneWithPath(); 
	}

	protected void end() {
//		System.out.println("Adaptive Pursuit finished");
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
	}

	protected void interrupted() {
//    	System.out.println("DrivePathAdaptivePursuit interrupted");
		end();
	}
}