package org.usfirst.frc.team3310.robot.subsystems;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Ramp extends Subsystem
{
	private static Ramp instance;
	
	public static enum RampLatch { STOWED, DEPLOYED };
	public static enum RampPull { UP, DOWN };

	private Solenoid latch;
	private Solenoid pull;
	
	private double matchStartTimeSeconds;

	private Ramp() {
		try {
			latch = new Solenoid(RobotMap.RAMP_LATCH_PCM_ID);
			pull = new Solenoid(RobotMap.RAMP_PULL_PCM_ID);
		}
		catch (Exception e) {
			System.err.println("An error occurred in the Ramp constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}
	
	public static Ramp getInstance() {
		if(instance == null) {
			instance = new Ramp();
		}
		return instance;
	}

	public void setLatchPosition(RampLatch state) {
		if(state == RampLatch.DEPLOYED) {
			latch.set(true);
		}
		else if(state == RampLatch.STOWED) {
			latch.set(false);
		}
	}
		
	public void setPullPosition(RampPull state) {
		if(state == RampPull.DOWN) {
			pull.set(true);
		}
		else if(state == RampPull.UP) {
			pull.set(false);
		}
	}
	
	public double getRemainingTeleopSeconds() {
		return 135.0 - (Timer.getFPGATimestamp() - matchStartTimeSeconds);
	}
	
	public void setTeleopStartTime() {
		matchStartTimeSeconds = Timer.getFPGATimestamp();
	}
		
	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				
			}
			catch (Exception e) {
			}
		}
	}
}