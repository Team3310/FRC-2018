package org.usfirst.frc.team3310.robot.subsystems;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.RobotMap;
import org.usfirst.frc.team3310.utility.TalonSRXFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Forks extends Subsystem
{
	public static enum ForksLockState { STOWED, DEPLOYED };

	private static Forks instance;

	public static final double WINCH_SPEED = 1.0;

	private TalonSRX winch;
	private Solenoid lock1;
	private Solenoid lock2;
	
	private ForksLockState lockState;
	
	private Forks() {
		try {
			winch = TalonSRXFactory.createDefaultTalon(RobotMap.FORKS_WINCH_CAN_ID);
			winch.setNeutralMode(NeutralMode.Brake);
			winch.setInverted(false);
			
			lockState = ForksLockState.STOWED;
			lock1 = new Solenoid(RobotMap.FORKS_LOCK_1_PCM_ID);
			lock2 = new Solenoid(RobotMap.FORKS_LOCK_2_PCM_ID);
		}
		catch (Exception e) {
			System.err.println("An error occurred in the Forks constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}
	
	public void setWinchSpeed(double speed) {
		winch.set(ControlMode.PercentOutput, speed);
	}
		
	public static Forks getInstance() {
		if(instance == null) {
			instance = new Forks();
		}
		return instance;
	}
		
	public void setLockState(ForksLockState state) {
		lockState = state;
		if(state == ForksLockState.STOWED) {
			lock1.set(false);
			lock2.set(false);
		}
		else if(state == ForksLockState.DEPLOYED) {
			lock1.set(true);
			lock2.set(true);
		}
	}
	
	public ForksLockState getLockState() {
		return lockState;
	}

	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Winch Amps", winch.getOutputCurrent());
			}
			catch (Exception e) {
			}
		}
	}
}