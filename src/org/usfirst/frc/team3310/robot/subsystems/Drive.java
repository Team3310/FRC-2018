package org.usfirst.frc.team3310.robot.subsystems;

import java.util.ArrayList;
import java.util.Set;

import org.usfirst.frc.team3310.robot.Constants;
import org.usfirst.frc.team3310.robot.OI;
import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.RobotMap;
import org.usfirst.frc.team3310.utility.AdaptivePurePursuitController;
import org.usfirst.frc.team3310.utility.BHRDifferentialDrive;
import org.usfirst.frc.team3310.utility.BHRMathUtils;
import org.usfirst.frc.team3310.utility.ControlLoopable;
import org.usfirst.frc.team3310.utility.Kinematics;
import org.usfirst.frc.team3310.utility.MMTalonPIDController;
import org.usfirst.frc.team3310.utility.MPSoftwarePIDController;
import org.usfirst.frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;
import org.usfirst.frc.team3310.utility.MPTalonPIDController;
import org.usfirst.frc.team3310.utility.MPTalonPIDPathController;
import org.usfirst.frc.team3310.utility.PIDParams;
import org.usfirst.frc.team3310.utility.Path;
import org.usfirst.frc.team3310.utility.RigidTransform2d;
import org.usfirst.frc.team3310.utility.Rotation2d;
import org.usfirst.frc.team3310.utility.SoftwarePIDController;
import org.usfirst.frc.team3310.utility.TalonSRXEncoder;
import org.usfirst.frc.team3310.utility.Translation2d;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem implements ControlLoopable
{
	private static Drive instance;

	public static enum DriveControlMode { JOYSTICK, MP_STRAIGHT, MP_TURN, PID_TURN, HOLD, MANUAL, MP_PATH, MP_PATH_VELOCITY, MOTION_MAGIC, ADAPTIVE_PURSUIT };
	public static enum SpeedShiftState { HI, LO };
	public static enum ClimberState { DEPLOYED, RETRACTED };

	// One revolution of the wheel = Pi * D inches = 60/24 revs due to gears * 36/12 revs due mag encoder gear on ball shifter * 4096 ticks 
	public static final double ENCODER_TICKS_TO_INCHES = (36.0 / 12.0) * (60.0 / 24.0) * 4096.0 / (5.7 * Math.PI);  
	public static final double TRACK_WIDTH_INCHES = 24.56;  // 26.937;
	
	public static final double VOLTAGE_RAMP_RATE = 150;  // Volts per second

	// Motion profile max velocities and accel times
	public static final double MAX_TURN_RATE_DEG_PER_SEC = 320;
	public static final double MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC =  120;  //72;
	public static final double MP_AUTON_MAX_BOILER_STRAIGHT_VELOCITY_INCHES_PER_SEC =  200;  
	public static final double MP_AUTON_MAX_LO_GEAR_STRAIGHT_VELOCITY_INCHES_PER_SEC =  320;  
	public static final double MP_AUTON_MAX_HIGH_GEAR_STRAIGHT_VELOCITY_INCHES_PER_SEC =  400;  
	public static final double MP_AUTON_MAX_TURN_RATE_DEG_PER_SEC =  270;
	public static final double MP_AUTON_MAX_BOILER_TURN_RATE_DEG_PER_SEC =  400;
	public static final double MP_GEAR_DEPLOY_VELOCITY_INCHES_PER_SEC = 25;
	public static final double MP_GEAR_DEPLOY_FASTER_VELOCITY_INCHES_PER_SEC = 80;
	
	public static final double MP_STRAIGHT_T1 = 600;
	public static final double MP_STRAIGHT_T2 = 300;
	public static final double MP_TURN_T1 = 600;
	public static final double MP_TURN_T2 = 300;
	public static final double MP_MAX_TURN_T1 = 400;
	public static final double MP_MAX_TURN_T2 = 200;
	
	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();	

	private TalonSRXEncoder leftDrive1;
	private WPI_TalonSRX leftDrive2;
	private WPI_TalonSRX leftDrive3;

	private TalonSRXEncoder rightDrive1;
	private WPI_TalonSRX rightDrive2;
	private WPI_TalonSRX rightDrive3;

	private BHRDifferentialDrive m_drive;
	
	private boolean isRed = true;
	
	private double periodMs;

	// Pneumatics
	private Solenoid speedShift;

	// Input devices
	public static final int DRIVER_INPUT_JOYSTICK_ARCADE = 0;
	public static final int DRIVER_INPUT_JOYSTICK_TANK = 1;
	public static final int DRIVER_INPUT_JOYSTICK_CHEESY = 2;
	public static final int DRIVER_INPUT_XBOX_CHEESY = 3;
	public static final int DRIVER_INPUT_XBOX_ARCADE_LEFT = 4;
	public static final int DRIVER_INPUT_XBOX_ARCADE_RIGHT = 5;
	public static final int DRIVER_INPUT_WHEEL = 6;

	public static final double STEER_NON_LINEARITY = 0.5;
	public static final double MOVE_NON_LINEARITY = 1.0;
	
	public static final double STICK_DEADBAND = 0.02;

	private int m_moveNonLinear = 0;
	private int m_steerNonLinear = -3;

	private double m_moveScale = 1.0;
	private double m_steerScale = 1.0;

	private double m_moveInput = 0.0;
	private double m_steerInput = 0.0;

	private double m_moveOutput = 0.0;
	private double m_steerOutput = 0.0;

	private double m_moveTrim = 0.0;
	private double m_steerTrim = 0.0;

	private boolean isFinished;
	private DriveControlMode controlMode = DriveControlMode.JOYSTICK;
	
	private MPTalonPIDController mpStraightController;
//	private PIDParams mpStraightPIDParams = new PIDParams(0.1, 0, 0, 0.005, 0.03, 0.15);  // 4 colsons
	private PIDParams mpStraightPIDParams = new PIDParams(0.1, 0, 0, 0.005, 0.03, 0.03);  // 4 omni
	private PIDParams mpHoldPIDParams = new PIDParams(1, 0, 0, 0.0, 0.0, 0.0); 

	private MMTalonPIDController mmStraightController;
	private PIDParams mmStraightPIDParams = new PIDParams(0, 0, 0, 0.24);  
	
	private MPSoftwarePIDController mpTurnController; // p    i   d     a      v      g    izone
//	private PIDParams mpTurnPIDParams = new PIDParams(0.07, 0.00002, 0.5, 0.00025, 0.008, 0.0, 100);  // 4 colson wheels
	private PIDParams mpTurnPIDParams = new PIDParams(0.03, 0.00002, 0.4, 0.0004, 0.0030, 0.0, 100);  // 4 omni
	
	private SoftwarePIDController pidTurnController;
	private PIDParams pidTurnPIDParams = new PIDParams(0.04, 0.001, 0.4, 0, 0, 0.0, 100); //i=0.0008

	private MPTalonPIDPathController mpPathController;
	private PIDParams mpPathPIDParams = new PIDParams(0.1, 0, 0, 0.005, 0.03, 0.28, 100);  // 4 omni   g=.3


	private AdaptivePurePursuitController adaptivePursuitController;
	private PIDParams adaptivePursuitPIDParams = new PIDParams(0.1, 0.00, 1, 0.44); 
	
	private RigidTransform2d zeroPose = new RigidTransform2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
	private RigidTransform2d currentPose = zeroPose;
	private RigidTransform2d lastPose = zeroPose;
    private double left_encoder_prev_distance_ = 0;
    private double right_encoder_prev_distance_ = 0;

	private PigeonIMU gyroPigeon;
	private double[] yprPigeon = new double[3];
	private boolean useGyroLock;
	private double gyroLockAngleDeg;
	private double kPGyro = 0.04;
	private boolean isCalibrating = false;
	private double gyroOffsetDeg = 0;

	private Drive() {
		try {
			leftDrive1 = new TalonSRXEncoder(RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID, ENCODER_TICKS_TO_INCHES, false, FeedbackDevice.QuadEncoder);
			leftDrive2 = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_MOTOR2_CAN_ID);
			leftDrive3 = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_MOTOR3_CAN_ID);

			rightDrive1 = new TalonSRXEncoder(RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID, ENCODER_TICKS_TO_INCHES, true, FeedbackDevice.QuadEncoder);
			rightDrive2 = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_MOTOR2_CAN_ID);
			rightDrive3 = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_MOTOR3_CAN_ID);
			
			gyroPigeon = new PigeonIMU(rightDrive2);
			
//			leftDrive1.clearStickyFaults(TalonSRXEncoder.TIMEOUT_MS);
			leftDrive1.setSensorPhase(true);   // Encoder on ball shifter spins opposite direction due to gears
//			leftDrive1.configClosedloopRamp(VOLTAGE_RAMP_RATE, TalonSRXEncoder.TIMEOUT_MS);
			leftDrive1.setNeutralMode(NeutralMode.Brake);
//			leftDrive1.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
//			leftDrive1.enableVoltageCompensation(true);
//			leftDrive1.configNominalOutputForward(0.0, TalonSRXEncoder.TIMEOUT_MS);
//			leftDrive1.configNominalOutputReverse(0.0, TalonSRXEncoder.TIMEOUT_MS);
//			leftDrive1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
//			leftDrive1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);
			leftDrive1.setSafetyEnabled(false);
//	        if (leftDrive1.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
//	            Driver.reportError("Could not detect left drive encoder encoder!", false);
//	        }
			
			leftDrive2.set(ControlMode.Follower, RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);
			leftDrive2.setNeutralMode(NeutralMode.Brake);
			leftDrive2.setSafetyEnabled(false);

			leftDrive3.set(ControlMode.Follower, RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);
			leftDrive3.setNeutralMode(NeutralMode.Brake);
			leftDrive3.setSafetyEnabled(false);
			
//			rightDrive1.clearStickyFaults(TalonSRXEncoder.TIMEOUT_MS);
//			rightDrive1.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
//			rightDrive1.enableVoltageCompensation(true);
//			rightDrive1.configNominalOutputForward(0.0, TalonSRXEncoder.TIMEOUT_MS);
//			rightDrive1.configNominalOutputReverse(0.0, TalonSRXEncoder.TIMEOUT_MS);
//			rightDrive1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
//			rightDrive1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);
			rightDrive1.setNeutralMode(NeutralMode.Brake);
			rightDrive1.setSafetyEnabled(false);
//	        if (rightDrive1.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
//	            DriverStation.reportError("Could not detect right drive encoder encoder!", false);
//	        }
			
			rightDrive2.set(ControlMode.Follower, RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			rightDrive2.setInverted(false);
			rightDrive2.setNeutralMode(NeutralMode.Brake);
			rightDrive2.setSafetyEnabled(false);

			rightDrive3.set(ControlMode.Follower, RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			rightDrive3.setInverted(false);
			rightDrive3.setNeutralMode(NeutralMode.Brake);
			rightDrive3.setSafetyEnabled(false);
							
			motorControllers.add(leftDrive1);
			motorControllers.add(rightDrive1);
			
			m_drive = new BHRDifferentialDrive(leftDrive1, rightDrive1);
			m_drive.setSafetyEnabled(false);
			
			speedShift = new Solenoid(RobotMap.DRIVETRAIN_SPEEDSHIFT_PCM_ID);
		}
		catch (Exception e) {
			System.err.println("An error occurred in the DriveTrain constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}
	
	public double getGyroAngleDeg() {
		return getGyroPigeonAngleDeg();
	}
	
	public double getGyroPigeonAngleDeg() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return -yprPigeon[0] + gyroOffsetDeg;
	}
			
	public void resetGyro() {
		gyroPigeon.setYaw(0, TalonSRXEncoder.TIMEOUT_MS);
		gyroPigeon.addFusedHeading(0, TalonSRXEncoder.TIMEOUT_MS);
	}
	
	public void resetEncoders() {
		mpStraightController.resetZeroPosition();
	}
	
	public void calibrateGyro() {
		gyroPigeon.enterCalibrationMode(CalibrationMode.Temperature, TalonSRXEncoder.TIMEOUT_MS);
	}
	
	public void endGyroCalibration() {
		if (isCalibrating == true) {
			isCalibrating = false;
		}
	}
	
	public void setGyroOffset(double offsetDeg) {
		gyroOffsetDeg = offsetDeg;
	}
	

	public void setStraightMM(double distanceInches, double maxVelocity, double maxAcceleration, boolean useGyroLock, boolean useAbsolute, double desiredAbsoluteAngle) {
		double yawAngle = useAbsolute ? BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), desiredAbsoluteAngle) : getGyroAngleDeg();
		mmStraightController.setPID(mmStraightPIDParams);
		mmStraightController.setMMStraightTarget(0, distanceInches, maxVelocity, maxAcceleration, useGyroLock, yawAngle, true); 
		setControlMode(DriveControlMode.MOTION_MAGIC);
	}
	
	public void setStraightMP(double distanceInches, double maxVelocity, boolean useGyroLock, boolean useAbsolute, double desiredAbsoluteAngle) {
		double yawAngle = useAbsolute ? BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), desiredAbsoluteAngle) : getGyroAngleDeg();
		mpStraightController.setPID(mpStraightPIDParams);
		mpStraightController.setMPStraightTarget(0, distanceInches, maxVelocity, MP_STRAIGHT_T1, MP_STRAIGHT_T2, useGyroLock, yawAngle, true); 
		setControlMode(DriveControlMode.MP_STRAIGHT);
	}
	
	public void setStraightMPCached(String key, boolean useGyroLock, boolean useAbsolute, double desiredAbsoluteAngle) {
		double yawAngle = useAbsolute ? BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), desiredAbsoluteAngle) : getGyroAngleDeg();
		mpStraightController.setPID(mpStraightPIDParams);
		mpStraightController.setMPStraightTarget(key, useGyroLock, yawAngle, true); 
		setControlMode(DriveControlMode.MP_STRAIGHT);
	}
	
	public void setRelativeTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec, MP_TURN_T1, MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}
	
	public void setRelativeTurnMPCached(String key, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(key, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}
	
	public void setRelativeMaxTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec, MP_MAX_TURN_T1, MP_MAX_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}
	
	public void setAbsoluteTurnMP(double absoluteTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(), BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), absoluteTurnAngleDeg), turnRateDegPerSec, MP_TURN_T1, MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}
	
	public void setAbsoluteTurnMPCached(String key, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(key, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}
		
//	public void setPathMP(PathGenerator path) {
//		mpPathController.setPID(mpPathPIDParams);
//		mpPathController.setMPPathTarget(path); 
//		setControlMode(DriveControlMode.MP_PATH);
//	}
	
//	public void setPathVelocityMP(PathGenerator path) {
//		mpPathVelocityController.setPID(mpPathPIDParams);
//		mpPathVelocityController.setMPPathTarget(path); 
//		setControlMode(DriveControlMode.MP_PATH_VELOCITY);
//	}
	
    public void setPathAdaptivePursuit(Path path, boolean reversed) {
    	currentPose = zeroPose;
    	lastPose = zeroPose;
        left_encoder_prev_distance_ = 0;
        right_encoder_prev_distance_ = 0;
        adaptivePursuitController.setPID(adaptivePursuitPIDParams);
    	adaptivePursuitController.setPath(Constants.kPathFollowingLookahead, Constants.kPathFollowingMaxAccel, path, reversed, 0.25); 
		setControlMode(DriveControlMode.ADAPTIVE_PURSUIT);
    }

    public void setDriveHold(boolean status) {
		if (status) {
			setControlMode(DriveControlMode.HOLD);
		}
		else {
			setControlMode(DriveControlMode.JOYSTICK);
		}
	}
    
    public void updatePose() {
        double left_distance = leftDrive1.getPositionWorld();
        double right_distance = rightDrive1.getPositionWorld();
        Rotation2d gyro_angle = Rotation2d.fromDegrees(-getGyroAngleDeg());
        lastPose = currentPose;
        currentPose = generateOdometryFromSensors(left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }
    
    public RigidTransform2d generateOdometryFromSensors(double left_encoder_delta_distance, double right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        return Kinematics.integrateForwardKinematics(lastPose, left_encoder_delta_distance, right_encoder_delta_distance, current_gyro_angle);
    }
	
    /**
     * Path Markers are an optional functionality that name the various
     * Waypoints in a Path with a String. This can make defining set locations
     * much easier.
     * 
     * @return Set of Strings with Path Markers that the robot has crossed.
     */
    public synchronized Set<String> getPathMarkersCrossed() {
        if (controlMode != DriveControlMode.ADAPTIVE_PURSUIT) {
            return null;
        } else {
            return adaptivePursuitController.getMarkersCrossed();
        }
    }

    public synchronized void setControlMode(DriveControlMode controlMode) {
 		this.controlMode = controlMode;
		if (controlMode == DriveControlMode.JOYSTICK) {
//			leftDrive1.changeControlMode(ControlMode.PercentVbus);
//			rightDrive1.changeControlMode(ControlMode.PercentVbus);
		}
		else if (controlMode == DriveControlMode.MANUAL) {
//			leftDrive1.changeControlMode(ControlMode.PercentVbus);
//			rightDrive1.changeControlMode(ControlMode.PercentVbus);
		}
		else if (controlMode == DriveControlMode.HOLD) {
			mpStraightController.setPID(mpHoldPIDParams);
//			leftDrive1.changeControlMode(ControlMode.Position);
			leftDrive1.setPosition(0);
			leftDrive1.set(ControlMode.Position, 0);
//			rightDrive1.changeControlMode(ControlMode.Position);
			rightDrive1.setPosition(0);
			rightDrive1.set(0);
		}
		isFinished = false;
	}
	
	public synchronized void controlLoopUpdate() {
		if (controlMode == DriveControlMode.JOYSTICK) {
			driveWithJoystick();
		}
		else if (!isFinished) {
			if (controlMode == DriveControlMode.MP_STRAIGHT) {
				isFinished = mpStraightController.controlLoopUpdate(getGyroAngleDeg()); 
			}
			else if (controlMode == DriveControlMode.MP_TURN) {
				isFinished = mpTurnController.controlLoopUpdate(getGyroAngleDeg()); 
			}
			else if (controlMode == DriveControlMode.PID_TURN) {
				isFinished = pidTurnController.controlLoopUpdate(getGyroAngleDeg()); 
			}
			else if (controlMode == DriveControlMode.MP_PATH) {
				isFinished = mpPathController.controlLoopUpdate(getGyroAngleDeg()); 
			}
//			else if (controlMode == DriveControlMode.MP_PATH_VELOCITY) {
//				isFinished = mpPathVelocityController.controlLoopUpdate(getGyroAngleDeg()); 
//			}
			else if (controlMode == DriveControlMode.MOTION_MAGIC) {
				isFinished = mpStraightController.controlLoopUpdate(getGyroAngleDeg()); 
			}
			else if (controlMode == DriveControlMode.ADAPTIVE_PURSUIT) {
				updatePose();
				isFinished = adaptivePursuitController.controlLoopUpdate(currentPose); 
			}
		}
	}
	
	public void setSpeed(double speed) {
		if (speed == 0) {
			setControlMode(DriveControlMode.JOYSTICK);
		}
		else {
			setControlMode(DriveControlMode.MANUAL);
			rightDrive1.set(ControlMode.PercentOutput, -speed);
			leftDrive1.set(ControlMode.PercentOutput, speed);
		}
	}
	
	public void setGyroLock(boolean useGyroLock, boolean snapToAbsolute0or180) {
		if (snapToAbsolute0or180) {
			gyroLockAngleDeg = BHRMathUtils.adjustAccumAngleToClosest180(getGyroAngleDeg());
		}
		else {
			gyroLockAngleDeg = getGyroAngleDeg();
		}
		this.useGyroLock = useGyroLock;
	}

	public void driveWithJoystick() {
		if(m_drive == null) return;

		m_moveInput = OI.getInstance().getDriverController().getLeftYAxis();
		m_steerInput = OI.getInstance().getDriverController().getRightXAxis();
		
		boolean isShift = OI.getInstance().getDriverController().getLeftBumperButton();
		if (isShift) {
			speedShift.set(true);
		}
		else {
			speedShift.set(false);
		}		
		
		m_moveOutput = adjustForSensitivity(m_moveScale, m_moveTrim,
					m_moveInput, m_moveNonLinear, MOVE_NON_LINEARITY);
		m_steerOutput = adjustForSensitivity(m_steerScale, m_steerTrim,
				m_steerInput, m_steerNonLinear, STEER_NON_LINEARITY);

		if (useGyroLock) {
			double yawError = gyroLockAngleDeg - getGyroAngleDeg();
			m_steerOutput = kPGyro * yawError;
		}

		m_drive.arcadeDrive(m_moveOutput, m_steerOutput);	
	}

	private boolean inDeadZone(double input) {
		boolean inDeadZone;
		if (Math.abs(input) < STICK_DEADBAND) {
			inDeadZone = true;
		} else {
			inDeadZone = false;
		}
		return inDeadZone;
	}

	public double adjustForSensitivity(double scale, double trim,
			double steer, int nonLinearFactor, double wheelNonLinearity) {
		if (inDeadZone(steer))
			return 0;

		steer += trim;
		steer *= scale;
		steer = limitValue(steer);

		int iterations = Math.abs(nonLinearFactor);
		for (int i = 0; i < iterations; i++) {
			if (nonLinearFactor > 0) {
				steer = nonlinearStickCalcPositive(steer, wheelNonLinearity);
			} else {
				steer = nonlinearStickCalcNegative(steer, wheelNonLinearity);
			}
		}
		return steer;
	}

	private double limitValue(double value) {
		if (value > 1.0) {
			value = 1.0;
		} else if (value < -1.0) {
			value = -1.0;
		}
		return value;
	}

	private double nonlinearStickCalcPositive(double steer,
			double steerNonLinearity) {
		return Math.sin(Math.PI / 2.0 * steerNonLinearity * steer)
				/ Math.sin(Math.PI / 2.0 * steerNonLinearity);
	}

	private double nonlinearStickCalcNegative(double steer,
			double steerNonLinearity) {
		return Math.asin(steerNonLinearity * steer)
				/ Math.asin(steerNonLinearity);
	}

	public void setShiftState(SpeedShiftState state) {
		if(state == SpeedShiftState.HI) {
			speedShift.set(true);
		}
		else if(state == SpeedShiftState.LO) {
			speedShift.set(false);
		}
	}

	public synchronized boolean isFinished() {
		return isFinished;
	}
	
	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}
	
	@Override
	public void setPeriodMs(long periodMs) {
		mmStraightController = new MMTalonPIDController(periodMs, mmStraightPIDParams, motorControllers);
		mpStraightController = new MPTalonPIDController(periodMs, mpStraightPIDParams, motorControllers);
		mpTurnController = new MPSoftwarePIDController(periodMs, mpTurnPIDParams, motorControllers);
		pidTurnController = new SoftwarePIDController(pidTurnPIDParams, motorControllers);
		mpPathController = new MPTalonPIDPathController(periodMs, mpPathPIDParams, motorControllers);
		adaptivePursuitController = new AdaptivePurePursuitController(periodMs, adaptivePursuitPIDParams, motorControllers);
		this.periodMs = periodMs;
	}
	
	public double getPeriodMs() {
		return periodMs;
	}
	
	public boolean isRed() {
		return isRed;
	}
	
	public void setIsRed(boolean status) {
		isRed = status;
	}
	
	public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    public static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Drive Right Position Inches", rightDrive1.getPositionWorld());
				SmartDashboard.putNumber("Drive Left Position Inches", leftDrive1.getPositionWorld());
				SmartDashboard.putNumber("Drive Left 1 Amps", Robot.pdp.getCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID));
				SmartDashboard.putNumber("Drive Left 2 Amps", Robot.pdp.getCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR2_CAN_ID));
				SmartDashboard.putNumber("Drive Left 3 Amps", Robot.pdp.getCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR3_CAN_ID));
				SmartDashboard.putNumber("Drive Right 1 Amps", Robot.pdp.getCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID));
				SmartDashboard.putNumber("Drive Right 2 Amps", Robot.pdp.getCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR2_CAN_ID));
				SmartDashboard.putNumber("Drive Right 3 Amps", Robot.pdp.getCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR3_CAN_ID));
				SmartDashboard.putNumber("Yaw Angle Pigeon Deg", getGyroPigeonAngleDeg());
//				SmartDashboard.putNumber("Steer Output", m_steerOutput);
//				SmartDashboard.putNumber("Move Output", m_moveOutput);
//				SmartDashboard.putNumber("Steer Input", m_steerInput);
//				SmartDashboard.putNumber("Move Input", m_moveInput);
			}
			catch (Exception e) {
				System.err.println("Drivetrain update status error");
			}
		}
		else {
			
		}
	}	
	
	public static Drive getInstance() {
		if(instance == null) {
			instance = new Drive();
		}
		return instance;
	}

}