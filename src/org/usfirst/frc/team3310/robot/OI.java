package org.usfirst.frc.team3310.robot;

import org.usfirst.frc.team3310.controller.IHandController;
import org.usfirst.frc.team3310.controller.XboxController;
import org.usfirst.frc.team3310.robot.commands.DriveSpeedShift;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.subsystems.Drive;

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
		
		//Smart Dashboard
		Button driveMP = new InternalButton();
		driveMP.whenPressed(new DriveStraightMP(70, Drive.MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, false, 0));
		SmartDashboard.putData("Drive Straight MP", driveMP);
	
		m_driverXbox.getLeftBumperButton();
	}
	
	public static OI getInstance() {
		if(instance == null) {
			instance = new OI();
		}
		return instance;
	}

	public IHandController getDriverController() {
		return m_driverXbox;
	}

	public IHandController getOperatorController() {
		return m_operatorXbox;
	}

}

