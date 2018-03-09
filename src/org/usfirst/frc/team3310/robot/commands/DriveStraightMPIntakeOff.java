package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class DriveStraightMPIntakeOff extends Command
{
	private double distanceInches, maxVelocityInchesPerSec, desiredAbsoluteAngle;
	private boolean useGyroLock, useAbsolute;

	public DriveStraightMPIntakeOff(double distanceInches, double maxVelocityInchesPerSec, boolean useGyroLock, boolean useAbsolute, double desiredAbsoluteAngle) {
		requires(Robot.drive);
		this.distanceInches = distanceInches;
		this.maxVelocityInchesPerSec = maxVelocityInchesPerSec;
		this.desiredAbsoluteAngle = desiredAbsoluteAngle;
		this.useGyroLock = useGyroLock;
		this.useAbsolute = useAbsolute;
	}

	protected void initialize() {
		Robot.drive.setStraightMP(distanceInches, maxVelocityInchesPerSec, useGyroLock, useAbsolute, desiredAbsoluteAngle);
	}

	protected void execute() {
	}

	protected boolean isFinished() {
		return Robot.intake.getFrontRightVEXIntakeSensor() ||  Robot.intake.getFrontLeftVEXIntakeSensor() || Robot.intake.getFrontIRIntakeSensor() || Robot.drive.isFinished(); 
	}

	protected void end() {
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
	}

	protected void interrupted() {
    	System.out.println("DriveStraightMPIntakeOff interrupted");
		end();
	}
}