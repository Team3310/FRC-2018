package org.usfirst.frc.team3310.robot;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team3310.buttons.XBoxDPadTriggerButton;
import org.usfirst.frc.team3310.buttons.XBoxTriggerButton;
import org.usfirst.frc.team3310.controller.XboxController;
import org.usfirst.frc.team3310.robot.commands.DriveGyroReset;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveSpeedShift;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorAutoZero;
import org.usfirst.frc.team3310.robot.commands.ElevatorClimb;
import org.usfirst.frc.team3310.robot.commands.ElevatorResetZero;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetMode;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetSpeed;
import org.usfirst.frc.team3310.robot.commands.ElevatorSpeedShift;
import org.usfirst.frc.team3310.robot.commands.FlipperFlip;
import org.usfirst.frc.team3310.robot.commands.IntakeCubeAndLift;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeed;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeedFrontSensorOff;
import org.usfirst.frc.team3310.robot.commands.RampSetPullPosition;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Elevator.ElevatorControlMode;
import org.usfirst.frc.team3310.robot.subsystems.Flipper.FlipperSide;
import org.usfirst.frc.team3310.robot.subsystems.Intake;
import org.usfirst.frc.team3310.robot.subsystems.Ramp.RampLatch;
import org.usfirst.frc.team3310.robot.subsystems.Ramp.RampPull;
import org.usfirst.frc.team3310.utility.Path;
import org.usfirst.frc.team3310.utility.Path.Waypoint;
import org.usfirst.frc.team3310.utility.Translation2d;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.InternalButton;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	private static OI instance;
	
	private XboxController m_driverXbox;
	private XboxController m_operatorXbox;
	
	private OI() {
		
		// Driver controller
		m_driverXbox = new XboxController(RobotMap.DRIVER_JOYSTICK_1_USB_ID);
	
        JoystickButton shiftDrivetrain = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.LEFT_BUMPER_BUTTON);
        shiftDrivetrain.whenPressed(new DriveSpeedShift(Drive.SpeedShiftState.HI));
        shiftDrivetrain.whenReleased(new DriveSpeedShift(Drive.SpeedShiftState.LO));

		// Operator controller
		m_operatorXbox = new XboxController(RobotMap.OPERATOR_JOYSTICK_1_USB_ID);
		
        JoystickButton intakeLoad = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.RIGHT_BUMPER_BUTTON);
        intakeLoad.whenPressed(new IntakeSetSpeed(Intake.INTAKE_LOAD_SPEED));
        intakeLoad.whenReleased(new IntakeSetSpeed(0.0));
		
        JoystickButton intakeEject = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.LEFT_BUMPER_BUTTON);
        intakeEject.whenPressed(new IntakeSetSpeed(Intake.INTAKE_EJECT_SPEED));
        intakeEject.whenReleased(new IntakeSetSpeed(0.0));
		
        XBoxTriggerButton intakeLoadSlow = new XBoxTriggerButton(m_operatorXbox, XBoxTriggerButton.RIGHT_TRIGGER);
        intakeLoadSlow.whenPressed(new IntakeSetSpeed(Intake.INTAKE_LOAD_SLOW_SPEED));
        intakeLoadSlow.whenReleased(new IntakeSetSpeed(0.0));
		
        XBoxTriggerButton intakeEjectSlow = new XBoxTriggerButton(m_operatorXbox, XBoxTriggerButton.LEFT_TRIGGER);
        intakeEjectSlow.whenPressed(new IntakeSetSpeed(Intake.INTAKE_EJECT_SLOW_SPEED));
        intakeEjectSlow.whenReleased(new IntakeSetSpeed(0.0));
		
        XBoxDPadTriggerButton elevatorShiftHi = new XBoxDPadTriggerButton(m_operatorXbox, XBoxDPadTriggerButton.UP);
        elevatorShiftHi.whenPressed(new ElevatorSpeedShift(Elevator.SpeedShiftState.HI));

        XBoxDPadTriggerButton elevatorClimb = new XBoxDPadTriggerButton(m_operatorXbox, XBoxDPadTriggerButton.DOWN);
        elevatorClimb.whenPressed(new ElevatorClimb());
        elevatorClimb.whenReleased(new ElevatorSetMode(ElevatorControlMode.JOYSTICK_PID));

        XBoxDPadTriggerButton elevatorJoystickManualMode = new XBoxDPadTriggerButton(m_operatorXbox, XBoxDPadTriggerButton.LEFT);
        elevatorJoystickManualMode.whenPressed(new ElevatorSetMode(ElevatorControlMode.JOYSTICK_MANUAL));

        XBoxDPadTriggerButton rampPullDown = new XBoxDPadTriggerButton(m_operatorXbox, XBoxDPadTriggerButton.RIGHT);
        rampPullDown.whenPressed(new RampSetPullPosition(RampPull.DOWN));

        JoystickButton elevatorPidMode = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.BACK_BUTTON);
        elevatorPidMode.whenPressed(new ElevatorSetMode(ElevatorControlMode.JOYSTICK_PID));

        JoystickButton elevatorReset = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.START_BUTTON);
        elevatorReset.whenPressed(new ElevatorResetZero());

//        JoystickButton elevatorMinPosition = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.A_BUTTON);
//        elevatorMinPosition.whenPressed(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));

		JoystickButton intakeCube = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.A_BUTTON);
		intakeCube.whenPressed(new IntakeSetSpeedFrontSensorOff(Intake.INTAKE_LOAD_SPEED));

        JoystickButton elevatorMaxPosition = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.Y_BUTTON);
        elevatorMaxPosition.whenPressed(new ElevatorSetPositionMP(Elevator.MAX_POSITION_INCHES));

        JoystickButton elevatorScalePosition = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.B_BUTTON);
        elevatorScalePosition.whenPressed(new ElevatorSetPositionMP(Elevator.SCALE_LOW_POSITION_INCHES));

        JoystickButton elevatorSwitchPosition = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.X_BUTTON);
        elevatorSwitchPosition.whenPressed(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));

//        JoystickButton elevatorScalePosition = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.B_BUTTON);
//        elevatorScalePosition.whenPressed(new ElevatorSetPositionPID(30));
//
//        JoystickButton elevatorSwitchPosition = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.X_BUTTON);
//        elevatorSwitchPosition.whenPressed(new ElevatorSetPositionPID(10));

        //Smart Dashboard
        Button intakeCubeSD = new InternalButton();
        intakeCubeSD.whenPressed(new IntakeSetSpeedFrontSensorOff(Intake.INTAKE_LOAD_SPEED));
        SmartDashboard.putData("Intake Cube", intakeCube);
        
        Button autoZero = new InternalButton();
        autoZero.whenPressed(new ElevatorAutoZero(false));
        SmartDashboard.putData("Elevator Auto Zero", autoZero);
        
        Button elevatorMinPositionSD = new InternalButton();
        elevatorMinPositionSD.whenPressed(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
        SmartDashboard.putData("Elevator Min Position", elevatorMinPositionSD);
        
        Button elevatorSwitchPositionSD = new InternalButton();
        elevatorSwitchPositionSD.whenPressed(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));
        SmartDashboard.putData("Elevator Switch Position", elevatorSwitchPositionSD);
        
        Button elevatorScaleLowPositionSD = new InternalButton();
        elevatorScaleLowPositionSD.whenPressed(new ElevatorSetPositionMP(Elevator.SCALE_LOW_POSITION_INCHES));
        SmartDashboard.putData("Elevator Scale Low Position", elevatorScaleLowPositionSD);
        
        Button elevatorScaleHighPositionSD = new InternalButton();
        elevatorScaleHighPositionSD.whenPressed(new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES));
        SmartDashboard.putData("Elevator Scale High Position", elevatorScaleHighPositionSD);
        
        Button elevatorMaxPositionSD = new InternalButton();
        elevatorMaxPositionSD.whenPressed(new ElevatorSetPositionMP(Elevator.MAX_POSITION_INCHES));
        SmartDashboard.putData("Elevator Max Position", elevatorMaxPositionSD);
        
        Button elevatorLo = new InternalButton();
        elevatorLo.whenPressed(new ElevatorSpeedShift(Elevator.SpeedShiftState.LO));
        SmartDashboard.putData("Elevator Lo Shift", elevatorLo);
        
        Button elevatorHi = new InternalButton();
        elevatorHi.whenPressed(new ElevatorSpeedShift(Elevator.SpeedShiftState.HI));
        SmartDashboard.putData("Elevator Hi Shift", elevatorHi);
        
        Button elevatorButtonUp = new InternalButton();
        elevatorButtonUp.whenPressed(new ElevatorSetSpeed(Elevator.TEST_SPEED_UP));
        elevatorButtonUp.whenReleased(new ElevatorSetSpeed(0.0));
        SmartDashboard.putData("Elevator Up", elevatorButtonUp);
        
        Button elevatorButtonDown = new InternalButton();;
        elevatorButtonDown.whenPressed(new ElevatorSetSpeed(Elevator.TEST_SPEED_DOWN));
        elevatorButtonDown.whenReleased(new ElevatorSetSpeed(0.0));
        SmartDashboard.putData("Elevator Down", elevatorButtonDown);

		Button flipLeft = new InternalButton();
		flipLeft.whenPressed(new FlipperFlip(FlipperSide.LEFT));
		SmartDashboard.putData("Flip Cube Left", flipLeft);
		
		Button flipRight = new InternalButton();
		flipRight.whenPressed(new FlipperFlip(FlipperSide.RIGHT));
		SmartDashboard.putData("Flip Cube Right", flipRight);
		
		Button intakeOn = new InternalButton();
		intakeOn.whenPressed(new IntakeSetSpeed(Intake.INTAKE_LOAD_SPEED));
		SmartDashboard.putData("Intake On", intakeOn);
		
		Button intakeOff = new InternalButton();
		intakeOff.whenPressed(new IntakeSetSpeed(0.0));
		SmartDashboard.putData("Intake Off", intakeOff);
		
		Button driveMP = new InternalButton();
		driveMP.whenPressed(new DriveStraightMP(70, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, false, 0));
		SmartDashboard.putData("Drive Straight MP", driveMP);
		
		// Center back around switch to scale
//		List<Waypoint> waypoints = new ArrayList<>();
//        waypoints.add(new Waypoint(new Translation2d(0, 0), 65.0));
//        waypoints.add(new Waypoint(new Translation2d(10, 0), 65.0));
//        Path.addCircleArc(waypoints, 40.0, 45.0, 20, "hopperSensorOn");
//        waypoints.add(new Waypoint(new Translation2d(90, 90), 65.0));
//        waypoints.add(new Waypoint(new Translation2d(100, 95), 65.0));
//        waypoints.add(new Waypoint(new Translation2d(110, 97), 65.0));
//        waypoints.add(new Waypoint(new Translation2d(130, 95), 65.0));
//        waypoints.add(new Waypoint(new Translation2d(256, 80), 65.0));

        // Center back around switch to scale
 		List<Waypoint> waypoints = new ArrayList<>();
		waypoints.add(new Waypoint(new Translation2d(0, 0), 65.0));
		waypoints.add(new Waypoint(new Translation2d(10, 0), 65.0));
		Path.addCircleArc(waypoints, 40.0, 45.0, 20, "hopperSensorOn");
		waypoints.add(new Waypoint(new Translation2d(90, 90), 65.0));
		waypoints.add(new Waypoint(new Translation2d(100, 95), 65.0));
		waypoints.add(new Waypoint(new Translation2d(110, 97), 65.0));
		waypoints.add(new Waypoint(new Translation2d(130, 95), 65.0));
		waypoints.add(new Waypoint(new Translation2d(160, 95), 65.0));
		waypoints.add(new Waypoint(new Translation2d(210, 40), 65.0));
		waypoints.add(new Waypoint(new Translation2d(210, 0), 65.0));
		
//        List<Waypoint> waypoints = new ArrayList<>();
//        waypoints.add(new Waypoint(new Translation2d(-96, 0), 50.0));
//        waypoints.add(new Waypoint(new Translation2d(0, 0), 35.0));
//        waypoints.add(new Waypoint(new Translation2d(-96, 0), 50.0));
//        waypoints.add(new Waypoint(new Translation2d(-61, 0), 50.0));
//        waypoints.add(new Waypoint(new Translation2d(-84.93, -10.16), 50.0));
        
//        waypoints.add(new Waypoint(new Translation2d(-41, 0), 35.0));
//        waypoints.add(new Waypoint(new Translation2d(-65, -23), 35.0));
//        waypoints.add(new Waypoint(new Translation2d(-70, -39), 35.0));
        
//        waypoints.add(new Waypoint(new Translation2d(-29, 0), 40.0));
//        Path.addCircleArc(waypoints, -30.0, 45.0, 10, null);
//        waypoints.add(new Waypoint(new Translation2d(-68.6, -26.5), 40.0));
		
		Button driveAP = new InternalButton();
		driveAP.whenPressed(new DrivePathAdaptivePursuit(new Path(waypoints), false));
		SmartDashboard.putData("Drive Adaptive Pursuit", driveAP);

		Button gyroReset = new InternalButton();
		gyroReset.whenPressed(new DriveGyroReset());
		SmartDashboard.putData("Gyro Reset", gyroReset);

		m_driverXbox.getLeftBumperButton();
	}
	
	public static OI getInstance() {
		if(instance == null) {
			instance = new OI();
		}
		return instance;
	}

	public XboxController getDriverController() {
		return m_driverXbox;
	}

	public XboxController getOperatorController() {
		return m_operatorXbox;
	}

}

