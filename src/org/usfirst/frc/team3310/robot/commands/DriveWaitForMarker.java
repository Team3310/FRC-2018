package org.usfirst.frc.team3310.robot.commands;

import java.util.Set;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Flipper.FlipperState;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWaitForMarker extends Command
{
	private String targetMarker;
	private double timeout;
	private double startTime;
	
	public DriveWaitForMarker(String targetMarker, double timeout) {
		this.targetMarker = targetMarker;
		this.timeout = timeout;
	}

	@Override
	protected void initialize() {
		setTimeout(timeout);
		startTime = System.currentTimeMillis();
		System.out.println("Start marker = " + targetMarker + ", time = " + System.currentTimeMillis());
	}

	@Override
	protected void execute() {		
	}

	@Override
	protected boolean isFinished() {
		if (isTimedOut()) {
			System.out.println("IsTimedOut marker = " + targetMarker);
			return true;
		}
		
		Set<String> markers = Robot.drive.getPathMarkersCrossed();
		if (markers == null) {
			System.out.println("null marker = " + targetMarker);
			return false;
		}
		
		for (String currentMarker : markers) {
			if (currentMarker.equals(targetMarker)) {
				System.out.println("Got to marker = " + targetMarker + ", delta time = " + (System.currentTimeMillis() - startTime) + ", time = " + System.currentTimeMillis());
				return true;
			}
		}
		
		return false;
	}

	@Override
	protected void end() {
		
	}

	@Override
	protected void interrupted() {
			
	}
}