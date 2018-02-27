package org.usfirst.frc.team3310.robot.subsystems;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem
{
	private static Intake instance;

	public static final double INTAKE_REAR_EJECT_SPEED = 1.0;
	public static final double INTAKE_LOAD_SPEED = 0.8;
	public static final double INTAKE_LOAD_SLOW_SPEED = 0.4;
	public static final double INTAKE_EJECT_SPEED = -0.8;
	public static final double INTAKE_EJECT_SLOW_SPEED = -0.4;
	public static final double INTAKE_ADJUST_SPEED = 0.3;

	private TalonSRX leftArm;
	private TalonSRX rightArm;

	// Sensors
	private DigitalInput frontIRIntakeSensor;
	private DigitalInput frontVEXIntakeSensor;
	private DigitalInput backIntakeSensor;
	
	private Intake() {
		try {
			leftArm = new TalonSRX(RobotMap.INTAKE_LEFT);
			leftArm.setNeutralMode(NeutralMode.Brake);
			leftArm.setInverted(true);
			
			rightArm = new TalonSRX(RobotMap.INTAKE_RIGHT);
			rightArm.setNeutralMode(NeutralMode.Brake);

			frontIRIntakeSensor = new DigitalInput(RobotMap.INTAKE_FRONT_IR_SENSOR_DIO_ID);
			frontVEXIntakeSensor = new DigitalInput(RobotMap.INTAKE_FRONT_VEX_SENSOR_DIO_ID);
			backIntakeSensor = new DigitalInput(RobotMap.INTAKE_BACK_IR_SENSOR_DIO_ID);						
		}
		catch (Exception e) {
			System.err.println("An error occurred in the Intake constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}
	
	public void setSpeed(double speed) {
		leftArm.set(ControlMode.PercentOutput, speed);
		rightArm.set(ControlMode.PercentOutput, -speed);
	}
		
	public static Intake getInstance() {
		if(instance == null) {
			instance = new Intake();
		}
		return instance;
	}

	public boolean getFrontIRIntakeSensor() {
		return frontIRIntakeSensor.get();
	}
	
	public boolean getFrontVEXIntakeSensor() {
		return !frontVEXIntakeSensor.get();   
	}
	
	public boolean getBackIntakeSensor() {
		return backIntakeSensor.get();
	}
	
	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putBoolean("Intake Front VEX Sensor", getFrontVEXIntakeSensor());
				SmartDashboard.putBoolean("Intake Front IR Sensor", getFrontIRIntakeSensor());
				SmartDashboard.putBoolean("Intake Back Sensor", getBackIntakeSensor());
			}
			catch (Exception e) {
			}
		}
	}
}