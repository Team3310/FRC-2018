package org.usfirst.frc.team3310.robot.subsystems;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.Robot.OperationMode;
import org.usfirst.frc.team3310.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Ramp extends Subsystem
{
	private static Ramp instance;
	
	public static enum RampLatch { STOWED, DEPLOYED };
	public static enum RampPull { UP, DOWN };

	private Solenoid latch;
	private Solenoid pull1;
	private Solenoid pull2;
	
	private double matchStartTimeSeconds;
	private OperationMode operationMode;

	private Ramp() {
		try {
			latch = new Solenoid(RobotMap.RAMP_LATCH_PCM_ID);
			pull1 = new Solenoid(RobotMap.RAMP_PULL1_PCM_ID);
			pull2 = new Solenoid(RobotMap.RAMP_PULL2_PCM_ID);
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
			pull1.set(true);
			pull2.set(true);
		}
		else if(state == RampPull.UP) {
			pull1.set(false);
			pull2.set(false);
		}
	}
	
	public double getRemainingTeleopSeconds() {
		return 135.0 - (Timer.getFPGATimestamp() - matchStartTimeSeconds);
	}
	
	public void setTeleopStartTime() {
		matchStartTimeSeconds = Timer.getFPGATimestamp();
	}
	
	public OperationMode getOperationMode() {
		return operationMode;
	}

	public void setOperationMode(OperationMode operationMode) {
		this.operationMode = operationMode;
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