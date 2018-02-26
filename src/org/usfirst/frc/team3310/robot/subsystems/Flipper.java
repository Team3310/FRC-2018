package org.usfirst.frc.team3310.robot.subsystems;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flipper extends Subsystem
{
	private static Flipper instance;
	
	public static enum FlipperSide { RIGHT, LEFT };
	public static enum FlipperState { RETRACTED, DEPLOYED };

	private Solenoid leftFlipper;
	private Solenoid rightFlipper;


	private Flipper() {
		try {
			leftFlipper = new Solenoid(RobotMap.LEFT_FLIPPER_PCM_ID);
			rightFlipper = new Solenoid(RobotMap.RIGHT_FLIPPER_PCM_ID);
			
		}
		catch (Exception e) {
			System.err.println("An error occurred in the Flipper constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}
	
	public static Flipper getInstance() {
		if(instance == null) {
			instance = new Flipper();
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
	    	System.out.println("Flipper right deploy, time = " + System.currentTimeMillis());
		}
		else if(state == FlipperState.RETRACTED) {
			rightFlipper.set(false);
	    	System.out.println("Flipper right retract");
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
			}
		}
	}
}