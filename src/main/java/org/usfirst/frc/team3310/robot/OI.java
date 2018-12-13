package org.usfirst.frc.team3310.robot;

import org.usfirst.frc.team3310.controller.GameController;
import org.usfirst.frc.team3310.controller.Logitech;
import org.usfirst.frc.team3310.controller.Playstation;
import org.usfirst.frc.team3310.controller.XBox;
import org.usfirst.frc.team3310.paths.test.CenterTest;
import org.usfirst.frc.team3310.robot.commands.DriveChaseCube;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetEncoders;
import org.usfirst.frc.team3310.robot.commands.DriveResetGyro;
import org.usfirst.frc.team3310.robot.commands.DriveSpeedShift;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorAutoZero;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetMode;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetSpeed;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;
import org.usfirst.frc.team3310.robot.commands.ElevatorSpeedShift;
import org.usfirst.frc.team3310.robot.commands.ForksIncrementWinchSpeed;
import org.usfirst.frc.team3310.robot.commands.ForksSetLock;
import org.usfirst.frc.team3310.robot.commands.ForksSetWinchSpeed;
import org.usfirst.frc.team3310.robot.commands.IntakeCubeAndLift;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeed;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeedFrontSensorOff;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Elevator.ElevatorControlMode;
import org.usfirst.frc.team3310.robot.subsystems.Elevator.ElevatorSpeedShiftState;
import org.usfirst.frc.team3310.robot.subsystems.Forks;
import org.usfirst.frc.team3310.robot.subsystems.Forks.ForksLockState;
import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	private static OI instance;
	
	
	private GameController m_driver;
	private GameController m_operator;

	private OI() {
		// Driver controller
		m_driver = new GameController(RobotMap.pilot, new Logitech());
		m_operator = new GameController(RobotMap.copilot, new Playstation());
	

    	Button shiftDrivetrain = m_driver.getLeftBumper();
		shiftDrivetrain.whenPressed(new DriveSpeedShift(Drive.DriveSpeedShiftState.HI));
		shiftDrivetrain.whenReleased(new DriveSpeedShift(Drive.DriveSpeedShiftState.LO));

 		Button forksDeploy = m_driver.getShareButton();
		forksDeploy.whenPressed(new ForksSetLock(ForksLockState.DEPLOYED));

        Button forksStow = m_driver.getOptionsButton();
		forksStow.whenPressed(new ForksSetLock(ForksLockState.STOWED));

        Button winchUp = m_driver.getDPadUp();
        winchUp.whenPressed(new ForksSetWinchSpeed(Forks.WINCH_SPEED));
		winchUp.whenReleased(new ForksSetWinchSpeed(0));

        Button winchDown = m_driver.getDPadDown();
		winchDown.whenPressed(new ForksSetWinchSpeed(-Forks.WINCH_SPEED));
		winchDown.whenReleased(new ForksSetWinchSpeed(0));

		Button winchIncrementHoldPlus = m_driver.getDPadLeft();
		winchIncrementHoldPlus.whenPressed(new ForksIncrementWinchSpeed(Forks.WINCH_HOLD_INCREMENT));

        Button winchIncrementHoldNeg = m_driver.getDPadRight();
		winchIncrementHoldNeg.whenPressed(new ForksIncrementWinchSpeed(-Forks.WINCH_HOLD_INCREMENT));

        // Operator controller		
        Button intakeLoad = m_operator.getRightBumper();
        intakeLoad.whenPressed(new IntakeSetSpeed(Intake.INTAKE_EJECT_SPEED));
        intakeLoad.whenReleased(new IntakeSetSpeed(0.0));
		
        Button intakeEject = m_operator.getLeftBumper();
        intakeEject.whenPressed(new IntakeSetSpeed(Intake.INTAKE_LOAD_SPEED));
        intakeEject.whenReleased(new IntakeSetSpeed(Intake.INTAKE_HOLD_SPEED));
		
        Button intakeLoadSlow = m_operator.getRightTrigger();
        intakeLoadSlow.whenPressed(new IntakeSetSpeed(Intake.INTAKE_EJECT_SLOW_SPEED));
        intakeLoadSlow.whenReleased(new IntakeSetSpeed(0.0));
		
        Button intakeEjectSlow = m_operator.getLeftTrigger();
        intakeEjectSlow.whenPressed(new IntakeSetSpeed(Intake.INTAKE_LOAD_SLOW_SPEED));
        intakeEjectSlow.whenReleased(new IntakeSetSpeed(Intake.INTAKE_HOLD_SPEED));
		
        Button elevatorShiftHi = m_operator.getDPadUp();
        elevatorShiftHi.whenPressed(new ElevatorSpeedShift(Elevator.ElevatorSpeedShiftState.HI));

        Button elevatorShiftLo = m_operator.getDPadDown();
        elevatorShiftLo.whenPressed(new ElevatorSpeedShift(ElevatorSpeedShiftState.LO));

        Button elevatorJoystickManualMode = m_operator.getDPadLeft();
        elevatorJoystickManualMode.whenPressed(new ElevatorSetMode(ElevatorControlMode.JOYSTICK_MANUAL));

        Button elevatorPidMode = m_operator.getStartButton();
        elevatorPidMode.whenPressed(new ElevatorSetMode(ElevatorControlMode.JOYSTICK_PID));

        Button elevatorReset = m_operator.getOptionsButton();
        elevatorReset.whenPressed(new ElevatorSetZero(Elevator.ZERO_POSITION_INCHES)); 

 		Button intakeCube = m_operator.getButtonA();
		intakeCube.whenPressed(new IntakeCubeAndLift());

        Button elevatorMaxPosition = m_operator.getButtonY();
        elevatorMaxPosition.whenPressed(new ElevatorSetPositionMP(Elevator.MAX_POSITION_INCHES));

        Button elevatorScalePosition = m_operator.getButtonB();
        elevatorScalePosition.whenPressed(new ElevatorSetPositionMP(Elevator.SCALE_LOW_POSITION_INCHES));

        Button elevatorSwitchPosition = m_operator.getButtonX();
        elevatorSwitchPosition.whenPressed(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES)); 

    // Shuffleboard
       SmartDashboard.putData("CameraTrack", new DriveChaseCube());
       SmartDashboard.putData("Forks Deploy", new ForksSetLock(ForksLockState.DEPLOYED));
       SmartDashboard.putData("Forks Retract", new ForksSetLock(ForksLockState.STOWED));

       SmartDashboard.putData("Elevator Auto Zero", new ElevatorAutoZero(false));
       SmartDashboard.putData("Elevator Min Position", new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
       SmartDashboard.putData("Elevator Switch Position", new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));
       SmartDashboard.putData("Elevator Scale Low Position", new ElevatorSetPositionMP(Elevator.SCALE_LOW_POSITION_INCHES));
       SmartDashboard.putData("Elevator Scale High Position", new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES));
       SmartDashboard.putData("Elevator Max Position", new ElevatorSetPositionMP(Elevator.MAX_POSITION_INCHES));
       SmartDashboard.putData("Elevator Lo Shift", new ElevatorSpeedShift(Elevator.ElevatorSpeedShiftState.LO));
       SmartDashboard.putData("Elevator Hi Shift", new ElevatorSpeedShift(Elevator.ElevatorSpeedShiftState.HI));
       SmartDashboard.putData("Elevator Manual Up", new ElevatorSetSpeed(Elevator.TEST_SPEED_UP));
       SmartDashboard.putData("Elevator Manual Down", new ElevatorSetSpeed(Elevator.TEST_SPEED_DOWN));
	   SmartDashboard.putData("Elevator Reset Encoder", new ElevatorSetZero(Elevator.ZERO_POSITION_INCHES));

	   SmartDashboard.putData("Intake On", new IntakeSetSpeed(Intake.INTAKE_LOAD_SPEED));
	   SmartDashboard.putData("Intake Off", new IntakeSetSpeed(0.0));
       SmartDashboard.putData("Intake Cube", new IntakeSetSpeedFrontSensorOff(Intake.INTAKE_LOAD_SPEED));

       SmartDashboard.putData("Drive Straight MP", new DriveStraightMP(72, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, false, 0));
	   SmartDashboard.putData("Drive Adaptive Pursuit", new DrivePathAdaptivePursuit(new CenterTest()));

	   SmartDashboard.putData("Drive Reset Gyro ", new DriveResetGyro());
	   SmartDashboard.putData("Drive Reset Encoder", new DriveResetEncoders());

        // Smart Dashboard
//        Button rampLatchClose = new InternalButton();
//        rampLatchClose.whenPressed(new RampSetLatchPosition(RampLatch.STOWED));
//        SmartDashboard.putData("Ramp Latch Close", rampLatchClose);
//        
//        Button rampPullRetract = new InternalButton();
//        rampPullRetract.whenPressed(new RampSetPullPosition(RampPull.UP));
//        SmartDashboard.putData("Ramp Pull Retract", rampPullRetract);
//
//        Button intakeCubeSD = new InternalButton();
//        intakeCubeSD.whenPressed(new IntakeSetSpeedFrontSensorOff(Intake.INTAKE_LOAD_SPEED));
//        SmartDashboard.putData("Intake Cube", intakeCube);
//        
//        Button autoZero = new InternalButton();
//        autoZero.whenPressed(new ElevatorAutoZero(false));
//        SmartDashboard.putData("Elevator Auto Zero", autoZero);
//        
//        Button elevatorMinPositionSD = new InternalButton();
//        elevatorMinPositionSD.whenPressed(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
//        SmartDashboard.putData("Elevator Min Position", elevatorMinPositionSD);
//        
//        Button elevatorSwitchPositionSD = new InternalButton();
//        elevatorSwitchPositionSD.whenPressed(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));
//        SmartDashboard.putData("Elevator Switch Position", elevatorSwitchPositionSD);
//        
//        Button elevatorScaleLowPositionSD = new InternalButton();
//        elevatorScaleLowPositionSD.whenPressed(new ElevatorSetPositionMP(Elevator.SCALE_LOW_POSITION_INCHES));
//        SmartDashboard.putData("Elevator Scale Low Position", elevatorScaleLowPositionSD);
//        
//        Button elevatorScaleHighPositionSD = new InternalButton();
//        elevatorScaleHighPositionSD.whenPressed(new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES));
//        SmartDashboard.putData("Elevator Scale High Position", elevatorScaleHighPositionSD);
//        
//        Button elevatorMaxPositionSD = new InternalButton();
//        elevatorMaxPositionSD.whenPressed(new ElevatorSetPositionMP(Elevator.MAX_POSITION_INCHES));
//        SmartDashboard.putData("Elevator Max Position", elevatorMaxPositionSD);
//        
//        Button elevatorLo = new InternalButton();
//        elevatorLo.whenPressed(new ElevatorSpeedShift(Elevator.ElevatorSpeedShiftState.LO));
//        SmartDashboard.putData("Elevator Lo Shift", elevatorLo);
//        
//        Button elevatorHi = new InternalButton();
//        elevatorHi.whenPressed(new ElevatorSpeedShift(Elevator.ElevatorSpeedShiftState.HI));
//        SmartDashboard.putData("Elevator Hi Shift", elevatorHi);
//        
//        Button elevatorButtonUp = new InternalButton();
//        elevatorButtonUp.whenPressed(new ElevatorSetSpeed(Elevator.TEST_SPEED_UP));
//        elevatorButtonUp.whenReleased(new ElevatorSetSpeed(0.0));
//        SmartDashboard.putData("Elevator Up", elevatorButtonUp);
//        
//        Button elevatorButtonDown = new InternalButton();;
//        elevatorButtonDown.whenPressed(new ElevatorSetSpeed(Elevator.TEST_SPEED_DOWN));
//        elevatorButtonDown.whenReleased(new ElevatorSetSpeed(0.0));
//        SmartDashboard.putData("Elevator Down", elevatorButtonDown);
//
//		Button flipLeft = new InternalButton();
//		flipLeft.whenPressed(new FlipperFlip(FlipperSide.LEFT, FlipperState.DEPLOYED));
//		SmartDashboard.putData("Flip Cube Left Deployed", flipLeft);
//		
//		Button flipRight = new InternalButton();
//		flipRight.whenPressed(new FlipperFlip(FlipperSide.RIGHT, FlipperState.DEPLOYED));
//		SmartDashboard.putData("Flip Cube Right Deployed", flipRight);
//		
//		Button flipRightRetract = new InternalButton();
//		flipRightRetract.whenPressed(new FlipperFlip(FlipperSide.RIGHT, FlipperState.RETRACTED));
//		SmartDashboard.putData("Flip Cube Right Retracted", flipRightRetract);
//		
//		Button flipLeftRetract = new InternalButton();
//		flipLeftRetract.whenPressed(new FlipperFlip(FlipperSide.LEFT, FlipperState.RETRACTED));
//		SmartDashboard.putData("Flip Cube Left Retracted", flipLeftRetract);
//		
//		Button intakeOn = new InternalButton();
//		intakeOn.whenPressed(new IntakeSetSpeed(Intake.INTAKE_LOAD_SPEED));
//		SmartDashboard.putData("Intake On", intakeOn);
//		
//		Button intakeOff = new InternalButton();
//		intakeOff.whenPressed(new IntakeSetSpeed(0.0));
//		SmartDashboard.putData("Intake Off", intakeOff);
//		
//		Button driveMP = new InternalButton();
//		driveMP.whenPressed(new DriveStraightMP(72, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, false, 0));
//		SmartDashboard.putData("Drive Straight MP", driveMP);

		// Center back around switch to scale
//		List<Waypoint> waypoints = new ArrayList<>();
//      waypoints.add(new Waypoint(new Translation2d(0, 0), 65.0));
//      waypoints.add(new Waypoint(new Translation2d(10, 0), 65.0));
//      Path.addCircleArc(waypoints, 40.0, 45.0, 20, "hopperSensorOn");
//      waypoints.add(new Waypoint(new Translation2d(90, 90), 65.0));
//      waypoints.add(new Waypoint(new Translation2d(100, 95), 65.0));
//      waypoints.add(new Waypoint(new Translation2d(110, 97), 65.0));
//      waypoints.add(new Waypoint(new Translation2d(130, 95), 65.0));
//      waypoints.add(new Waypoint(new Translation2d(256, 80), 65.0));

//      // Center back around switch to scale
// 		List<Waypoint> waypoints = new ArrayList<>();
//		waypoints.add(new Waypoint(new Translation2d(0, 0), 65.0));
//		waypoints.add(new Waypoint(new Translation2d(10, 0), 65.0));
//		Path.addCircleArc(waypoints, 40.0, 45.0, 20, "hopperSensorOn");
//		waypoints.add(new Waypoint(new Translation2d(90, 90), 65.0));
//		waypoints.add(new Waypoint(new Translation2d(100, 95), 65.0));
//		waypoints.add(new Waypoint(new Translation2d(110, 97), 65.0));
//		waypoints.add(new Waypoint(new Translation2d(130, 95), 65.0));
//		waypoints.add(new Waypoint(new Translation2d(160, 95), 65.0));
//		waypoints.add(new Waypoint(new Translation2d(210, 40), 65.0));
//		waypoints.add(new Waypoint(new Translation2d(210, 0), 65.0));
		
//      List<Waypoint> waypoints = new ArrayList<>();
//      waypoints.add(new Waypoint(new Translation2d(-96, 0), 50.0));
//      waypoints.add(new Waypoint(new Translation2d(0, 0), 35.0));
//      waypoints.add(new Waypoint(new Translation2d(-96, 0), 50.0));
//      waypoints.add(new Waypoint(new Translation2d(-61, 0), 50.0));
//      waypoints.add(new Waypoint(new Translation2d(-84.93, -10.16), 50.0));
        
//      waypoints.add(new Waypoint(new Translation2d(-41, 0), 35.0));
//      waypoints.add(new Waypoint(new Translation2d(-65, -23), 35.0));
//      waypoints.add(new Waypoint(new Translation2d(-70, -39), 35.0));
        
//      waypoints.add(new Waypoint(new Translation2d(-29, 0), 40.0));
//      Path.addCircleArc(waypoints, -30.0, 45.0, 10, null);
//      waypoints.add(new Waypoint(new Translation2d(-68.6, -26.5), 40.0));
		
        // Center back around switch to scale
// 		List<Waypoint> waypoints = new ArrayList<>();
//		waypoints.add(new Waypoint(new Translation2d(0, 0), 50.0));
//		waypoints.add(new Waypoint(new Translation2d(0, -30), 50.0, 30.0));
//		waypoints.add(new Waypoint(new Translation2d(100, -80), 50.0, 50.0));
//		waypoints.add(new Waypoint(new Translation2d(100, -160), 50.0, 20.0));
//		waypoints.add(new Waypoint(new Translation2d(85, -260), 50.0));

//		Button driveAP = new InternalButton();
//		driveAP.whenPressed(new DrivePathAdaptivePursuit(new CenterTest()));
//		SmartDashboard.putData("Drive Adaptive Pursuit", driveAP);
//
//		Button gyroReset = new InternalButton();
//		gyroReset.whenPressed(new DriveResetGyro());
//		SmartDashboard.putData("Gyro Reset", gyroReset);

//		m_driverXbox.getLeftBumperButton();
	}
	
	public static OI getInstance() {
		if(instance == null) {
			instance = new OI();
		}
		return instance;
	}

	public GameController getDriverController() {
		return m_driver;
	}

	public GameController getOperatorController() {
		return m_operator;
	}

}

