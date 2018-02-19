package org.usfirst.frc.team3310.robot.subsystems;

import java.util.ArrayList;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.RobotMap;
import org.usfirst.frc.team3310.utility.ControlLoopable;
import org.usfirst.frc.team3310.utility.MPTalonPIDController;
import org.usfirst.frc.team3310.utility.PIDParams;
import org.usfirst.frc.team3310.utility.TalonSRXEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem implements ControlLoopable
{
	private static Elevator instance;

	public static enum ElevatorControlMode { MOTION_PROFILE, JOYSTICK_PID, JOYSTICK_MANUAL, MANUAL };
	public static enum SpeedShiftState { HI, LO };

	// One revolution of the 30T Drive 1.880" PD pulley = Pi * PD inches = 36/24 revs due to pulleys * 34/24 revs due to gears * 36/12 revs due mag encoder gear on ball shifter * 4096 ticks 
	public static final double ENCODER_TICKS_TO_INCHES = (36.0 / 12.0) * (36.0 / 24.0) * (34.0 / 24.0) * 4096.0 / (1.88 * Math.PI);   
	
	// Defined speeds
	public static final double CLIMB_SPEED = -1.0;
	public static final double TEST_SPEED_UP = 0.3;
	public static final double TEST_SPEED_DOWN = -0.3;
	public static final double AUTO_ZERO_SPEED = -0.4;
	public static final double JOYSTICK_INCHES_PER_MS = 0.75;
	
	// Defined positions
	public static final double ZERO_POSITION_INCHES = -0.25;
	public static final double NEAR_ZERO_POSITION_INCHES = 3.0;
	public static final double MIN_POSITION_INCHES = 0.0;
	public static final double MAX_POSITION_INCHES = 83.0;

	public static final double SWITCH_POSITION_INCHES = 24.0;
	public static final double SCALE_LOW_POSITION_INCHES = 56.0;
	public static final double SCALE_HIGH_POSITION_INCHES = MAX_POSITION_INCHES;
	public static final double CLIMB_BAR_POSITION_INCHES = 70.0;
	public static final double CLIMB_HIGH_POSITION_INCHES = 10.0;
	public static final double CLIMB_ASSIST_POSITION_INCHES = 50.0;

	// Motion profile max velocities and accel times
	public static final double MP_MAX_VELOCITY_INCHES_PER_SEC =  100; 
	public static final double MP_T1 = 600;  // Fast = 300
	public static final double MP_T2 = 300;  // Fast = 150
	
	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();	

	private TalonSRXEncoder motor1;
	private TalonSRX motor2;
	private TalonSRX motor3;
	
	// Sensors
	private DigitalInput frontIntakeSensor;
	private DigitalInput backIntakeSensor;
	
	// PID controller and params
	private MPTalonPIDController mpController;

	public static int PID_SLOT = 0;
	public static int MP_SLOT = 1;

	private PIDParams mpPIDParams = new PIDParams(0.1, 0.0, 0.0, 0.0, 0.005, 0.0);  
	private PIDParams pidPIDParams = new PIDParams(0.035, 0.0, 0.0, 0.0, 0.0, 0.0);  
	public static final double KF_UP = 0.005;
	public static final double KF_DOWN = 0.0;
	public static final double PID_ERROR_INCHES = 1.0;
	private double periodMs;

	// Pneumatics
	private Solenoid speedShift;

	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 20.0;	
	private boolean isFinished;
	private ElevatorControlMode controlMode = ElevatorControlMode.JOYSTICK_MANUAL;
	private double targetPositionInchesPID = 0;
	private boolean firstMpPoint;
	
	private Elevator() {
		try {
			motor1 = new TalonSRXEncoder(RobotMap.ELEVATOR_MOTOR_1_CAN_ID, ENCODER_TICKS_TO_INCHES, false, FeedbackDevice.QuadEncoder);
			motor2 = new TalonSRX(RobotMap.ELEVATOR_MOTOR_2_CAN_ID);
			motor3 = new TalonSRX(RobotMap.ELEVATOR_MOTOR_3_CAN_ID);
			
//			motor1.setSensorPhase(true);
			motor1.setInverted(true);
			motor1.setNeutralMode(NeutralMode.Brake);
//			motor1.configMaxIntegralAccumulator(arg0, arg1, arg2);
//			motor1.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
//			motor1.enableVoltageCompensation(true);
//			motor1.configNominalOutputForward(0.0, TalonSRXEncoder.TIMEOUT_MS);
//			motor1.configNominalOutputReverse(0.0, TalonSRXEncoder.TIMEOUT_MS);
//			motor1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
//			motor1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);
//	        if (motor1.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
//	            Driver.reportError("Could not detect elevator motor 1 encoder encoder!", false);
//	        }
			
			motor2.set(ControlMode.Follower, RobotMap.ELEVATOR_MOTOR_1_CAN_ID);
			motor2.setNeutralMode(NeutralMode.Brake);
			motor2.setInverted(true);

			motor3.set(ControlMode.Follower, RobotMap.ELEVATOR_MOTOR_1_CAN_ID);
			motor3.setNeutralMode(NeutralMode.Brake);
			motor3.setInverted(true);
										
			motorControllers.add(motor1);
			
			speedShift = new Solenoid(RobotMap.ELEVATOR_SPEEDSHIFT_PCM_ID);
			frontIntakeSensor = new DigitalInput(RobotMap.INTAKE_FRONT_SENSOR_DIO_ID);
			backIntakeSensor = new DigitalInput(RobotMap.INTAKE_BACK_SENSOR_DIO_ID);						
		}
		catch (Exception e) {
			System.err.println("An error occurred in the DriveTrain constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}
		
	public void resetZeroPosition(double position) {
		mpController.resetZeroPosition(position);
	}	

	public void setSpeed(double speed) {
		this.controlMode = ElevatorControlMode.MANUAL;
		motor1.set(ControlMode.PercentOutput, speed);
	}
		
	public void setSpeedJoystick(double speed) {
		this.controlMode = ElevatorControlMode.JOYSTICK_MANUAL;
		motor1.set(ControlMode.PercentOutput, speed);
	}
		
	public void setPositionPID(double targetPositionInches) {
 		this.controlMode = ElevatorControlMode.JOYSTICK_PID;
 		
 		targetPositionInchesPID = limitPosition(targetPositionInches);
		double startPositionInches = motor1.getPositionWorld();
		mpController.setTarget(targetPositionInchesPID, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN); 
		isFinished = false;
	}
	
	public void setPositionMP(double targetPositionInches) {
		double startPositionInches = motor1.getPositionWorld();
		mpController.setMPTarget(startPositionInches, limitPosition(targetPositionInches), MP_MAX_VELOCITY_INCHES_PER_SEC, MP_T1, MP_T2); 
		isFinished = false;
		firstMpPoint = true;
 		this.controlMode = ElevatorControlMode.MOTION_PROFILE;
	}
	
	private double limitPosition(double targetPosition) {
		if (targetPosition < MIN_POSITION_INCHES) {
			return MIN_POSITION_INCHES;
		}
		else if (targetPosition > MAX_POSITION_INCHES) {
			return MAX_POSITION_INCHES;
		}
		
		return targetPosition;
	}
	
	public synchronized void controlLoopUpdate() {
		switch( controlMode ) {
			case JOYSTICK_PID: 
				controlPidWithJoystick();
				break;
			case JOYSTICK_MANUAL:
				controlManualWithJoystick();
				break;
			case MOTION_PROFILE: 
				if (!isFinished) {
					if (firstMpPoint) {
						mpController.setPIDSlot(MP_SLOT);
						firstMpPoint = false;
					}
					isFinished = mpController.controlLoopUpdate(); 
					if (isFinished) {
						mpController.setPIDSlot(PID_SLOT);
					}
					
				}
				break;
			default:
				break;
		}
	}
	
	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * JOYSTICK_INCHES_PER_MS;
		targetPositionInchesPID = targetPositionInchesPID + deltaPosition;
		setPositionPID(targetPositionInchesPID);
	}
	
	private void controlManualWithJoystick() {
		double joyStickSpeed = -Robot.oi.getOperatorController().getLeftYAxis();
		setSpeedJoystick(joyStickSpeed);
	}
	
	public void setShiftState(SpeedShiftState state) {
		if(state == SpeedShiftState.HI) {
			speedShift.set(true);
		}
		else if(state == SpeedShiftState.LO) {
			speedShift.set(false);
		}
	}

	public double getPositionInches() {
		return motor1.getPositionWorld();
	}
	
	public double getMotorCurrent() {
		return (Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID) + Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID) + Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID)) / 3;
	}
		
	public synchronized boolean isFinished() {
		return isFinished;
	}
	
	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}
	
	@Override
	public void setPeriodMs(long periodMs) {
		mpController = new MPTalonPIDController(periodMs, motorControllers);
		mpController.setPID(mpPIDParams, MP_SLOT);
		mpController.setPID(pidPIDParams, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
		this.periodMs = periodMs;
	}
	
	public double getPeriodMs() {
		return periodMs;
	}
	
	public boolean getFrontIntakeSensor() {
		return frontIntakeSensor.get();
	}
	
	public boolean getBackIntakeSensor() {
		return backIntakeSensor.get();
	}
	
	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Elevator Position Inches", motor1.getPositionWorld());
				SmartDashboard.putNumber("Elevator Motor 1 Amps", motor1.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Motor 2 Amps", motor1.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Motor 3 Amps", motor1.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Average Amps", getMotorCurrent());
				SmartDashboard.putNumber("Elevator Motor 1 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID));
				SmartDashboard.putNumber("Elevator Motor 2 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID));
				SmartDashboard.putNumber("Elevator Motor 3 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID));
				SmartDashboard.putBoolean("Intake Front Sensor", getFrontIntakeSensor());
				SmartDashboard.putBoolean("Intake Back Sensor", getBackIntakeSensor());
				SmartDashboard.putNumber("Elevator Target PID Position", targetPositionInchesPID);
			}
			catch (Exception e) {
				System.err.println("Elevator update status error");
			}
		}
	}	
	
	public static Elevator getInstance() {
		if(instance == null) {
			instance = new Elevator();
		}
		return instance;
	}

}