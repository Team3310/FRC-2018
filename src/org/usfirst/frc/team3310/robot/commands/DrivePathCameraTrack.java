package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class DrivePathCameraTrack extends Command
{
	private double straightVelocity;
	private double timeout;

	public DrivePathCameraTrack(double straightVelocity, double timeout) {
        this.straightVelocity = straightVelocity;
        this.timeout = timeout;
		requires(Robot.drive);
	}

	protected void initialize() {
		System.out.println("Start camera track");
		Robot.drive.setLimeLED(true);
		Robot.drive.setCameraTrack(straightVelocity);
		setTimeout(timeout);
	}

	protected void execute() {
	}

	protected boolean isFinished() {
		return isTimedOut(); 
	}

	protected void end() {
		System.out.println("camera track finished");
		Robot.drive.setLimeLED(false);
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
	}

	protected void interrupted() {
    	System.out.println("camera track interrupted");
		end();
	}
}