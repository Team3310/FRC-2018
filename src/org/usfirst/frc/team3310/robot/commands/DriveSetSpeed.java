package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class DriveSetSpeed extends Command
{
	private double speed;
	private double timeout;

	public DriveSetSpeed(double speed, double timeout) {
        this.speed = speed;
        this.timeout = timeout;
		requires(Robot.drive);
	}

	protected void initialize() {
		setTimeout(timeout);
		Robot.drive.setSpeed(speed);
	}

	protected void execute() {
	}

	protected boolean isFinished() {
		return isTimedOut(); 
	}

	protected void end() {
		Robot.drive.setSpeed(0);
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
	}

	protected void interrupted() {
		end();
	}
}