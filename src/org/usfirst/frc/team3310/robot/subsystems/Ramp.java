package org.usfirst.frc.team3310.robot.subsystems;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Ramp extends Subsystem
{
	private static Ramp instance;
	
	public static enum FlipperSide { RIGHT, LEFT };
	public static enum FlipperState { RETRACTED, DEPLOYED };

	private Solenoid leftFlipper;
	private Solenoid rightFlipper;


	private Ramp() {
		try {
			leftFlipper = new Solenoid(RobotMap.RAMP_LATCH_PCM_ID);
			rightFlipper = new Solenoid(RobotMap.RAMP_PULL_UP_PCM_ID);
			
		}
		catch (Exception e) {
			System.err.println("An error occurred in the Flipper constructor");
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

	public void setPosition(FlipperSide flipperSide, FlipperState state) {
		if(flipperSide == FlipperSide.RIGHT) {
			setRightPosition(state);
		}
		else {
			setLeftPosition(state);
		}
	}

	public void setRightPosition(FlipperState state) {
		if(state == FlipperState.DEPLOYED) {
			rightFlipper.set(true);
		}
		else if(state == FlipperState.RETRACTED) {
			rightFlipper.set(false);
		}
	}
	
	public FlipperState getRightFlipper() {
		return (rightFlipper.get() == true) ? FlipperState.DEPLOYED : FlipperState.RETRACTED;
	}
	
	public void setLeftPosition(FlipperState state) {
		if(state == FlipperState.DEPLOYED) {
			leftFlipper.set(true);
		}
		else if(state == FlipperState.RETRACTED) {
			leftFlipper.set(false);
		}
	}
	
	public FlipperState getLeftFlipper() {
		return (leftFlipper.get() == true) ? FlipperState.DEPLOYED : FlipperState.RETRACTED;
	}
	
	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				
			}
			catch (Exception e) {
				System.err.println("Flipper update status error");
			}
		}
	}
}