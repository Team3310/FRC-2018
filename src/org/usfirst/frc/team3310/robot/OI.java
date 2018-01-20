package org.usfirst.frc.team3310.robot;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team3310.controller.IHandController;
import org.usfirst.frc.team3310.controller.XboxController;
import org.usfirst.frc.team3310.robot.commands.DriveGyroReset;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveSpeedShift;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
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
		
		//Smart Dashboard
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
		SmartDashboard.putData("Drive Adaptive Pursuit 2", driveAP);

		Button gyroReset = new InternalButton();
		gyroReset.whenPressed(new DriveGyroReset());
		SmartDashboard.putData("Gyro Reset 2", gyroReset);

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

