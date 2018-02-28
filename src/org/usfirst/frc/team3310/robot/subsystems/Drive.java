package org.usfirst.frc.team3310.robot.subsystems;

import java.util.ArrayList;

import org.usfirst.frc.team3310.robot.Constants;
import org.usfirst.frc.team3310.robot.OI;
import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.RobotMap;
import org.usfirst.frc.team3310.utility.BHRDifferentialDrive;
import org.usfirst.frc.team3310.utility.BHRMathUtils;
import org.usfirst.frc.team3310.utility.Loop;
import org.usfirst.frc.team3310.utility.MPSoftwarePIDController;
import org.usfirst.frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;
import org.usfirst.frc.team3310.utility.MPTalonPIDController;
import org.usfirst.frc.team3310.utility.PIDParams;
import org.usfirst.frc.team3310.utility.SoftwarePIDController;
import org.usfirst.frc.team3310.utility.TalonSRXEncoder;
import org.usfirst.frc.team3310.utility.TalonSRXFactory;
import org.usfirst.frc.team3310.utility.control.Kinematics;
import org.usfirst.frc.team3310.utility.control.Lookahead;
import org.usfirst.frc.team3310.utility.control.Path;
import org.usfirst.frc.team3310.utility.control.PathFollower;
import org.usfirst.frc.team3310.utility.control.RobotState;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Rotation2d;
import org.usfirst.frc.team3310.utility.math.Twist2d;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem implements Loop
{
	private static Drive instance;

	public static enum DriveControlMode { JOYSTICK, MP_STRAIGHT, MP_TURN, PID_TURN, HOLD, MANUAL, ADAPTIVE_PURSUIT, VELOCITY_SETPOINT };
	public static enum DriveSpeedShiftState { HI, LO };
	public static enum ClimberState { DEPLOYED, RETRACTED };

	// One revolution of the wheel = Pi * D inches = 60/24 revs due to gears * 36/12 revs due mag encoder gear on ball shifter * 4096 ticks 
	public static final double ENCODER_TICKS_TO_INCHES = (36.0 / 12.0) * (60.0 / 24.0) * 4096.0 / (5.7 * Math.PI);  
	public static final double TRACK_WIDTH_INCHES = 24.56;  // 26.937;
	
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
	private TalonSRX leftDrive2;
	private TalonSRX leftDrive3;

	private TalonSRXEncoder rightDrive1;
	private TalonSRX rightDrive2;
	private TalonSRX rightDrive3;

	private BHRDifferentialDrive m_drive;
	
	private boolean isRed = true;
    private boolean mIsBrakeMode;
	
	private long periodMs = (long)(Constants.kLooperDt * 1000.0);
	
    protected Rotation2d mAngleAdjustment = Rotation2d.identity();

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
	private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK;
	
    private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;
    private static final int kLowGearVelocityControlSlot = 2;

    private MPTalonPIDController mpStraightController;
//	private PIDParams mpStraightPIDParams = new PIDParams(0.1, 0, 0, 0.005, 0.03, 0.15);  // 4 colsons
	private PIDParams mpStraightPIDParams = new PIDParams(0.05, 0, 0, 0.0008, 0.004, 0.03);  // 4 omni
	private PIDParams mpHoldPIDParams = new PIDParams(1, 0, 0, 0.0, 0.0, 0.0); 

	private MPSoftwarePIDController mpTurnController; // p    i   d     a      v      g    izone
//	private PIDParams mpTurnPIDParams = new PIDParams(0.07, 0.00002, 0.5, 0.00025, 0.008, 0.0, 100);  // 4 colson wheels
	private PIDParams mpTurnPIDParams = new PIDParams(0.03, 0.00002, 0.4, 0.0004, 0.0030, 0.0, 100);  // 4 omni
	
	private SoftwarePIDController pidTurnController;
	private PIDParams pidTurnPIDParams = new PIDParams(0.04, 0.001, 0.4, 0, 0, 0.0, 100); //i=0.0008

//	private PIDParams adaptivePursuitPIDParams = new PIDParams(0.1, 0.00, 1, 0.44); 
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;
    private RobotState mRobotState = RobotState.getInstance();
	
	private PigeonIMU gyroPigeon;
	private double[] yprPigeon = new double[3];
	private boolean useGyroLock;
	private double gyroLockAngleDeg;
	private double kPGyro = 0.04;
	private boolean isCalibrating = false;
	private double gyroOffsetDeg = 0;

    /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlMode state) {
        if (state == DriveControlMode.VELOCITY_SETPOINT || state == DriveControlMode.ADAPTIVE_PURSUIT) {
            return true;
        }
        return false;
    }

    /**
     * Check if the drive talons are configured for position control
     */
    protected static boolean usesTalonPositionControl(DriveControlMode state) {
        if (state == DriveControlMode.MP_STRAIGHT ||
                state == DriveControlMode.MP_TURN ||
                state == DriveControlMode.HOLD) {
            return true;
        }
        return false;
    }

    private Drive() {
		try {
			leftDrive1 = TalonSRXFactory.createTalonEncoder(RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID, ENCODER_TICKS_TO_INCHES, false, FeedbackDevice.QuadEncoder);
			leftDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR2_CAN_ID, RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);
			leftDrive3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR3_CAN_ID, RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);

			rightDrive1 = TalonSRXFactory.createTalonEncoder(RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID, ENCODER_TICKS_TO_INCHES, true, FeedbackDevice.QuadEncoder);
			rightDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR2_CAN_ID, RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			rightDrive3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR3_CAN_ID, RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			
			gyroPigeon = new PigeonIMU(rightDrive2);
			
			leftDrive1.setInverted(true);
			leftDrive1.setSensorPhase(false);   // Encoder on ball shifter spins opposite direction due to gears
//			leftDrive1.configClosedloopRamp(VOLTAGE_RAMP_RATE, TalonSRXEncoder.TIMEOUT_MS);
			leftDrive1.setNeutralMode(NeutralMode.Brake);
			leftDrive1.setSafetyEnabled(false);
//	        if (leftDrive1.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
//	            Driver.reportError("Could not detect left drive encoder encoder!", false);
//	        }
			
			leftDrive2.setInverted(true);
			leftDrive3.setInverted(true);
			
//			rightDrive1.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
//			rightDrive1.enableVoltageCompensation(true);
			rightDrive1.setSensorPhase(false);   // Encoder on ball shifter spins opposite direction due to gears
			rightDrive1.setNeutralMode(NeutralMode.Brake);
			rightDrive1.setSafetyEnabled(false);
			rightDrive1.setInverted(false);
//	        if (rightDrive1.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
//	            DriverStation.reportError("Could not detect right drive encoder encoder!", false);
//	        }
			
			rightDrive2.setInverted(false);
			rightDrive3.setInverted(false);
							
			motorControllers.add(leftDrive1);
			motorControllers.add(rightDrive1);
			
			m_drive = new BHRDifferentialDrive(leftDrive1, rightDrive1);
			m_drive.setSafetyEnabled(false);
			
			loadGains();
			
			speedShift = new Solenoid(RobotMap.DRIVETRAIN_SPEEDSHIFT_PCM_ID);
		}
		catch (Exception e) {
			System.err.println("An error occurred in the DriveTrain constructor");
		}
	}

    public synchronized void loadGains() {
        leftDrive1.setPIDFIZone(kLowGearVelocityControlSlot, 
        		Constants.kDriveLowGearVelocityKp, 
        		Constants.kDriveLowGearVelocityKi,
                Constants.kDriveLowGearVelocityKd, 
                Constants.kDriveLowGearVelocityKf,
                Constants.kDriveLowGearVelocityIZone);
        
        rightDrive1.setPIDFIZone(kLowGearVelocityControlSlot, 
        		Constants.kDriveLowGearVelocityKp, 
        		Constants.kDriveLowGearVelocityKi,
                Constants.kDriveLowGearVelocityKd, 
                Constants.kDriveLowGearVelocityKf,
                Constants.kDriveLowGearVelocityIZone);
        
        leftDrive1.setPIDFIZone(kHighGearVelocityControlSlot, 
        		Constants.kDriveHighGearVelocityKp, 
        		Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, 
                Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone);
        
        rightDrive1.setPIDFIZone(kHighGearVelocityControlSlot, 
        		Constants.kDriveHighGearVelocityKp, 
        		Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, 
                Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone);        
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
		gyroPigeon.setFusedHeading(0, TalonSRXEncoder.TIMEOUT_MS);
	}
	
	public void resetGyro(double value) {
		gyroPigeon.setYaw(value, TalonSRXEncoder.TIMEOUT_MS);
		gyroPigeon.setFusedHeading(value, TalonSRXEncoder.TIMEOUT_MS);
	}
	
    public synchronized Rotation2d getGyroAngle() {
        return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(getGyroAngleDeg()));
    }

    public synchronized void setGyroAngle(Rotation2d adjustment) {
    	resetGyro();
        mAngleAdjustment = adjustment;
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
		
	public void setStraightMP(double distanceInches, double maxVelocity, boolean useGyroLock, boolean useAbsolute, double desiredAbsoluteAngle) {
		double yawAngle = useAbsolute ? BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), desiredAbsoluteAngle) : getGyroAngleDeg();
		mpStraightController.setPID(mpStraightPIDParams, kLowGearPositionControlSlot);
		mpStraightController.setPIDSlot(kLowGearPositionControlSlot);
		mpStraightController.setMPStraightTarget(0, distanceInches, maxVelocity, MP_STRAIGHT_T1, MP_STRAIGHT_T2, useGyroLock, yawAngle, true); 
		setControlMode(DriveControlMode.MP_STRAIGHT);
	}
		
	public void setRelativeTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec, MP_TURN_T1, MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
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
	
    public void setDriveHold(boolean status) {
		if (status) {
			setControlMode(DriveControlMode.HOLD);
		}
		else {
			setControlMode(DriveControlMode.JOYSTICK);
		}
	}
    
    public synchronized void setControlMode(DriveControlMode controlMode) {
 		this.driveControlMode = controlMode;
 		if (controlMode == DriveControlMode.HOLD) {
			mpStraightController.setPID(mpHoldPIDParams, kLowGearPositionControlSlot);
			leftDrive1.setPosition(0);
			leftDrive1.set(ControlMode.Position, 0);
			rightDrive1.setPosition(0);
			rightDrive1.set(ControlMode.Position, 0);
		}
		setFinished(false);
	}
    
    public synchronized DriveControlMode getControlMode() {
    	return driveControlMode;
    }
	
	@Override
	public void onStart(double timestamp) {
		mpStraightController = new MPTalonPIDController(periodMs, motorControllers);
		mpStraightController.setPID(mpStraightPIDParams, kLowGearPositionControlSlot);
		mpTurnController = new MPSoftwarePIDController(periodMs, mpTurnPIDParams, motorControllers);
		pidTurnController = new SoftwarePIDController(pidTurnPIDParams, motorControllers);
	}

	@Override
	public void onStop(double timestamp) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onLoop(double timestamp) {
		synchronized (Drive.this) {
			DriveControlMode currentControlMode = getControlMode();

			if (currentControlMode == DriveControlMode.JOYSTICK) {
				driveWithJoystick();
			}
			else if (!isFinished()) {
				switch (currentControlMode) {			
					case MP_STRAIGHT :
						setFinished(mpStraightController.controlLoopUpdate(getGyroAngleDeg())); 
	                    break;
					case MP_TURN:
						setFinished(mpTurnController.controlLoopUpdate(getGyroAngleDeg())); 
	                    break;
					case PID_TURN:
						setFinished(pidTurnController.controlLoopUpdate(getGyroAngleDeg())); 
	                    break;
					case ADAPTIVE_PURSUIT:
	                    if (mPathFollower != null) {
	                        updatePathFollower(timestamp);
	                    }
	                    return;
	                default:
	                    System.out.println("Unknown drive control mode: " + currentControlMode);
	                    break;
                }
			}
			else {
				// hold in current state
			}
		}
	}
	
	public void setSpeed(double speed) {
		if (speed == 0) {
			setControlMode(DriveControlMode.JOYSTICK);
		}
		else {
			setControlMode(DriveControlMode.MANUAL);
			rightDrive1.set(ControlMode.PercentOutput, speed);
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

    /**
     * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
     */
    private void updatePathFollower(double timestamp) {
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        } else {
            updateVelocitySetpoint(0, 0);
        }
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || driveControlMode != DriveControlMode.ADAPTIVE_PURSUIT) {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                            Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
                            Constants.kPathStopSteeringDistance));
            driveControlMode = DriveControlMode.ADAPTIVE_PURSUIT;
            mCurrentPath = path;
        } else {
            setVelocitySetpoint(0, 0);
        }
    }

    /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        driveControlMode = DriveControlMode.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(driveControlMode)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
                    ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
//            leftDrive1.setVelocityWorld(left_inches_per_sec * scale);
//            rightDrive1.setVelocityWorld(right_inches_per_sec * scale);
            leftDrive1.set(inchesPerSecondToRpm(left_inches_per_sec * scale));
            rightDrive1.set(inchesPerSecondToRpm(right_inches_per_sec * scale));
            System.out.println("left vel = " + inchesPerSecondToRpm(left_inches_per_sec * scale) + ", right vel = " + inchesPerSecondToRpm(right_inches_per_sec * scale) + ", scale = " + scale);
        } else {
            System.out.println("Hit a bad velocity control state");
            leftDrive1.set(ControlMode.Velocity, 0);
            rightDrive1.set(ControlMode.Velocity, 0);
        }
    }

    /**
     * Configures talons for velocity control
     */
    private void configureTalonsForSpeedControl() {
        if (!usesTalonVelocityControl(driveControlMode)) {
            // We entered a velocity control state.
//        	leftDrive1.enableVoltageCompensation(true);
//        	leftDrive1.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
        	leftDrive1.selectProfileSlot(kHighGearVelocityControlSlot, TalonSRXEncoder.PID_IDX);
//        	leftDrive1.configNominalOutputForward(Constants.kDriveHighGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
//        	leftDrive1.configNominalOutputReverse(-Constants.kDriveHighGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
//        	leftDrive1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
//        	leftDrive1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);
            leftDrive1.configClosedloopRamp(Constants.kDriveHighGearVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
        	    	
//        	rightDrive1.enableVoltageCompensation(true);
//        	rightDrive1.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
        	rightDrive1.selectProfileSlot(kHighGearVelocityControlSlot, TalonSRXEncoder.PID_IDX);
//        	rightDrive1.configNominalOutputForward(Constants.kDriveHighGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
//        	rightDrive1.configNominalOutputReverse(-Constants.kDriveHighGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
//        	rightDrive1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
//        	rightDrive1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);
        	rightDrive1.configClosedloopRamp(Constants.kDriveHighGearVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);

        	setBrakeMode(true);
        }
    }

    public synchronized boolean isDoneWithPath() {
        if (driveControlMode == DriveControlMode.ADAPTIVE_PURSUIT && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (driveControlMode != DriveControlMode.ADAPTIVE_PURSUIT && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (driveControlMode != DriveControlMode.ADAPTIVE_PURSUIT && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            rightDrive1.setNeutralMode(NeutralMode.Brake);
            rightDrive2.setNeutralMode(NeutralMode.Brake);
            rightDrive3.setNeutralMode(NeutralMode.Brake);
            leftDrive1.setNeutralMode(NeutralMode.Brake);
            leftDrive2.setNeutralMode(NeutralMode.Brake);
            leftDrive3.setNeutralMode(NeutralMode.Brake);
        }
    }

    public void driveWithJoystick() {
		if(m_drive == null) return;

		m_moveInput = OI.getInstance().getDriverController().getLeftYAxis();
		m_steerInput = -OI.getInstance().getDriverController().getRightXAxis();
		
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

		m_drive.arcadeDrive(-m_moveOutput, -m_steerOutput);	
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

	public void setShiftState(DriveSpeedShiftState state) {
		if(state == DriveSpeedShiftState.HI) {
			speedShift.set(true);
		}
		else if(state == DriveSpeedShiftState.LO) {
			speedShift.set(false);
		}
	}

	public synchronized boolean isFinished() {
		return isFinished;
	}
	
	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
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
    
    public double getRightPositionInches() {
    	return rightDrive1.getPositionWorld();
    }

    public double getLeftPositionInches() {
    	return leftDrive1.getPositionWorld();
    }

    public double getRightVelocityInchesPerSec() {
    	return rightDrive1.getVelocityWorld();
    }

    public double getLeftVelocityInchesPerSec() {
    	return leftDrive1.getVelocityWorld();
    }

	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Drive Right Position Inches", rightDrive1.getPositionWorld());
				SmartDashboard.putNumber("Drive Left Position Inches", leftDrive1.getPositionWorld());
//				SmartDashboard.putNumber("Drive Left 1 Amps", Robot.pdp.getCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID));
//				SmartDashboard.putNumber("Drive Left 2 Amps", Robot.pdp.getCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR2_CAN_ID));
//				SmartDashboard.putNumber("Drive Left 3 Amps", Robot.pdp.getCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR3_CAN_ID));
//				SmartDashboard.putNumber("Drive Right 1 Amps", Robot.pdp.getCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID));
//				SmartDashboard.putNumber("Drive Right 2 Amps", Robot.pdp.getCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR2_CAN_ID));
//				SmartDashboard.putNumber("Drive Right 3 Amps", Robot.pdp.getCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR3_CAN_ID));
				SmartDashboard.putNumber("Yaw Angle Pigeon Deg", getGyroPigeonAngleDeg());
			}
			catch (Exception e) {
			}
		}
	}	
	
	public static Drive getInstance() {
		if(instance == null) {
			instance = new Drive();
		}
		return instance;
	}

}