package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveSpeedShiftState;
import org.usfirst.frc.team3310.utility.Path;

import edu.wpi.first.wpilibj.command.Command;

public class DrivePathAdaptivePursuit extends Command
{
	private Path path;

	public DrivePathAdaptivePursuit(Path path) {
		requires(Robot.drive);
		this.path = path;
	}

	protected void initialize() {
//		Robot.drive.setShiftState(SpeedShiftState.HI);
		Robot.drive.setPathAdaptivePursuit(path);
		System.out.println("Adaptive Pursuit start");
	}

	protected void execute() {
	}

	protected boolean isFinished() {
		return Robot.drive.isFinished(); 
	}

	protected void end() {
		System.out.println("Adaptive Pursuit finished");
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
	}

	protected void interrupted() {
    	System.out.println("DrivePathAdaptivePursuit interrupted");
		end();
	}
}