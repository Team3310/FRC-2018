/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3310.robot;

import java.util.HashMap;

import org.usfirst.frc.team3310.robot.commands.ElevatorAutoZero;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchLeft1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchLeft1ScaleLeft1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchLeft1ScaleRight1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchRight1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchRight1ScaleLeft1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchRight1ScaleRight1;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Flipper;
import org.usfirst.frc.team3310.robot.subsystems.Intake;
import org.usfirst.frc.team3310.robot.subsystems.Ramp;
import org.usfirst.frc.team3310.utility.Looper;
import org.usfirst.frc.team3310.utility.control.RobotState;
import org.usfirst.frc.team3310.utility.control.RobotStateEstimator;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static OI oi;
	
	// Declare subsystems
	public static final Drive drive = Drive.getInstance();
	public static final Elevator elevator = Elevator.getInstance();
	public static final Intake intake = Intake.getInstance();
	public static final Flipper flipper = Flipper.getInstance();
	public static final Ramp ramp = Ramp.getInstance();
	
	// Control looper
	public static final Looper controlLoop = new Looper();
	
	// Choosers
	private SendableChooser<OperationMode> operationModeChooser;
	private SendableChooser<AutonRouteChooser> autonTaskChooser;
    private Command autonomousCommand;

	public static enum OperationMode { TEST, PRACTICE, COMPETITION };
	public static OperationMode operationMode = OperationMode.TEST;

	// PDP
	public static final PowerDistributionPanel pdp = new PowerDistributionPanel();
	
	// State
    private RobotState mRobotState = RobotState.getInstance();

    public void zeroAllSensors() {
        drive.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
        drive.zeroSensors();
    }

	@Override
	public void robotInit() {
		setPeriod(Constants.kLooperDt * 2);
		System.out.println("Main loop period = " + getPeriod());
		
		oi = OI.getInstance();
		
    	controlLoop.register(drive);
    	controlLoop.register(elevator);
    	controlLoop.register(RobotStateEstimator.getInstance());
 
    	// Update default at competition!!!
    	operationModeChooser = new SendableChooser<OperationMode>();
	    operationModeChooser.addDefault("Practice", OperationMode.PRACTICE);
	    operationModeChooser.addObject("Competition", OperationMode.COMPETITION);
	    operationModeChooser.addObject("Test", OperationMode.TEST);
		SmartDashboard.putData("Operation Mode", operationModeChooser);

		autonTaskChooser = new SendableChooser<AutonRouteChooser>();
		
		AutonRouteChooser centerStartSwitch1Scale1 = new AutonRouteChooser();
		centerStartSwitch1Scale1.addLLL(new CenterStartToSwitchLeft1ScaleLeft1());
		centerStartSwitch1Scale1.addLRL(new CenterStartToSwitchLeft1ScaleRight1());
		centerStartSwitch1Scale1.addRLR(new CenterStartToSwitchRight1ScaleLeft1());
		centerStartSwitch1Scale1.addRRR(new CenterStartToSwitchRight1ScaleRight1());
		
		AutonRouteChooser centerStartSwitch1 = new AutonRouteChooser();
		centerStartSwitch1.addLLL(new CenterStartToSwitchLeft1());
		centerStartSwitch1.addLRL(new CenterStartToSwitchLeft1());
		centerStartSwitch1.addRLR(new CenterStartToSwitchRight1());
		centerStartSwitch1.addRRR(new CenterStartToSwitchRight1());
		
		autonTaskChooser.addDefault("Center Start Switch 1", centerStartSwitch1);
		autonTaskChooser.addObject("Center Start Switch 1 Scale 1", centerStartSwitch1Scale1);
		SmartDashboard.putData("Auton Tasks", autonTaskChooser);
		
		LiveWindow.setEnabled(false);
		LiveWindow.disableAllTelemetry();

		zeroAllSensors();

		drive.setLimeLED(true);
		drive.setLimeCameraMode(false);
	}  
	
	// Called every loop for all modes
	public void robotPeriodic() {
		updateStatus();
	}

	@Override
	public void disabledInit() {
		drive.setLimeLED(false);
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
        zeroAllSensors();
	}

	@Override
	public void autonomousInit() {
    	controlLoop.start();
    	drive.setIsRed(getAlliance().equals(Alliance.Red));
    	elevator.setShiftState(Elevator.ElevatorSpeedShiftState.HI);
    	elevator.resetZeroPosition(Elevator.ZERO_POSITION_INCHES);
        zeroAllSensors();

		drive.setLimeLED(true);
		drive.setLimeCameraMode(false);
		
		String gameMessage = m_ds.getGameSpecificMessage();
		while (gameMessage == null || gameMessage.length() < 3) {
			gameMessage = m_ds.getGameSpecificMessage();
		}

		AutonRouteChooser routeChooser = autonTaskChooser.getSelected();
		autonomousCommand = routeChooser.getRoute(gameMessage);

		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		operationMode = operationModeChooser.getSelected();
		
        controlLoop.start();
    	ramp.setTeleopStartTime();
    	ramp.setOperationMode(operationMode);
    	drive.endGyroCalibration();
    	elevator.setShiftState(Elevator.ElevatorSpeedShiftState.HI);
        zeroAllSensors();

		drive.setLimeLED(true);
		drive.setLimeCameraMode(false);
    	
    	if (operationMode != OperationMode.COMPETITION) {
    		Scheduler.getInstance().add(new ElevatorAutoZero(false));
    	}
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}
	
    public Alliance getAlliance() {
    	return m_ds.getAlliance();
    }
    
    public double getMatchTime() {
    	return m_ds.getMatchTime();
    }
    
    public void updateStatus() {
    	drive.updateStatus(operationMode);
    	intake.updateStatus(operationMode);
    	elevator.updateStatus(operationMode);
    	flipper.updateStatus(operationMode);
    	ramp.updateStatus(operationMode);
    }
    
    private class AutonRouteChooser {
    	
    	private HashMap<String, Command> commandMap = new HashMap<String, Command>(4);
    	
    	public AutonRouteChooser() {
    		
    	}
    	
    	public Command getRoute(String gameMessage) {
    		if (!commandMap.containsKey(gameMessage.toUpperCase())) {
    			System.out.println("Invalid game message");
    		}
    		return commandMap.get(gameMessage.toUpperCase());
    	}
    	
    	public void addLLL(Command route) {
    		commandMap.put("LLL", route);
    	}
    	
    	public void addLRL(Command route) {
    		commandMap.put("LRL", route);
    	}
    	
    	public void addRLR(Command route) {
    		commandMap.put("RLR", route);
    	}
    	
    	public void addRRR(Command route) {
    		commandMap.put("RRR", route);
    	}
    }

}
