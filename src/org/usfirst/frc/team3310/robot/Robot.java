/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3310.robot;

import java.util.HashMap;

import org.usfirst.frc.team3310.paths.auton.GoStraight;
import org.usfirst.frc.team3310.paths.auton.LeftStartToScaleLeft;
import org.usfirst.frc.team3310.paths.auton.LeftStartToScaleLeftV2;
import org.usfirst.frc.team3310.paths.auton.RightStartToScaleRight;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeft;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeft2;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeftNoIntake;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight2;
import org.usfirst.frc.team3310.paths.auton.SwitchLeft2ToScaleLeft;
import org.usfirst.frc.team3310.paths.auton.SwitchLeftToScaleLeft;
import org.usfirst.frc.team3310.paths.auton.SwitchRight2ToScaleRight;
import org.usfirst.frc.team3310.paths.auton.SwitchRightToScaleRight;
import org.usfirst.frc.team3310.robot.commands.ElevatorAutoZero;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchLeft1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchLeft1ScaleLeft1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchLeft1ScaleLeft2;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchLeft1ScaleRight1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchRight1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchRight1ScaleLeft1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchRight1ScaleRight1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchRight1ScaleRight2;
import org.usfirst.frc.team3310.robot.commands.auton.IntakeTest;
import org.usfirst.frc.team3310.robot.commands.auton.LeftStartToSwitchLeft1ScaleLeft1;
import org.usfirst.frc.team3310.robot.commands.auton.LeftStartToSwitchLeft1ScaleRight1;
import org.usfirst.frc.team3310.robot.commands.auton.LeftStartToSwitchRight1;
import org.usfirst.frc.team3310.robot.commands.auton.RightStartToSwitchLeft1;
import org.usfirst.frc.team3310.robot.commands.auton.RightStartToSwitchRight1ScaleLeft1;
import org.usfirst.frc.team3310.robot.commands.auton.RightStartToSwitchRight1ScaleRight1;
import org.usfirst.frc.team3310.robot.commands.auton.SideStartToSwitch3;
import org.usfirst.frc.team3310.robot.commands.auton.StartLeftCenterRightGoStraight;
import org.usfirst.frc.team3310.robot.commands.auton.StartToScale1Switch1Scale1;
import org.usfirst.frc.team3310.robot.commands.auton.StartToScale3;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveSpeedShiftState;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Flipper;
import org.usfirst.frc.team3310.robot.subsystems.Intake;
import org.usfirst.frc.team3310.robot.subsystems.Ramp;
import org.usfirst.frc.team3310.robot.subsystems.Ramp.RampPull;
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
    private RobotState robotState = RobotState.getInstance();

    public void zeroAllSensors() {
        drive.zeroSensors();
        robotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
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
	    operationModeChooser.addObject("Practice", OperationMode.PRACTICE);
	    operationModeChooser.addDefault("Competition", OperationMode.COMPETITION);
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
		
		AutonRouteChooser leftStartSwitch1Scale1 = new AutonRouteChooser();
		leftStartSwitch1Scale1.addLLL(new LeftStartToSwitchLeft1ScaleLeft1());
		leftStartSwitch1Scale1.addLRL(new LeftStartToSwitchLeft1ScaleRight1());
		leftStartSwitch1Scale1.addRLR(new LeftStartToSwitchRight1());
		leftStartSwitch1Scale1.addRRR(new LeftStartToSwitchRight1());
		
		AutonRouteChooser rightStartSwitch1Scale1 = new AutonRouteChooser();
		rightStartSwitch1Scale1.addLLL(new RightStartToSwitchLeft1());
		rightStartSwitch1Scale1.addLRL(new RightStartToSwitchLeft1());
		rightStartSwitch1Scale1.addRLR(new RightStartToSwitchRight1ScaleLeft1());
		rightStartSwitch1Scale1.addRRR(new RightStartToSwitchRight1ScaleRight1());
		
		AutonRouteChooser goStraight = new AutonRouteChooser();
		goStraight.addLLL(new StartLeftCenterRightGoStraight(new GoStraight()));
		goStraight.addLRL(new StartLeftCenterRightGoStraight(new GoStraight()));
		goStraight.addRLR(new StartLeftCenterRightGoStraight(new GoStraight()));
		goStraight.addRRR(new StartLeftCenterRightGoStraight(new GoStraight()));
		
		AutonRouteChooser leftStartScale3Cube = new AutonRouteChooser();
		leftStartScale3Cube.addLLL(new StartToScale3(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartScale3Cube.addRLR(new StartToScale3(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartScale3Cube.addLRL(new LeftStartToSwitchLeft1ScaleRight1());
		leftStartScale3Cube.addRRR(new LeftStartToSwitchRight1());

		AutonRouteChooser rightStartScale3Cube = new AutonRouteChooser();
		rightStartScale3Cube.addRRR(new StartToScale3(new RightStartToScaleRight(), new ScaleRightToSwitchRight(), new SwitchRightToScaleRight(), new ScaleRightToSwitchRight2(), new SwitchRight2ToScaleRight()));
		rightStartScale3Cube.addLRL(new StartToScale3(new RightStartToScaleRight(), new ScaleRightToSwitchRight(), new SwitchRightToScaleRight(), new ScaleRightToSwitchRight2(), new SwitchRight2ToScaleRight()));
		rightStartScale3Cube.addRLR(new RightStartToSwitchRight1ScaleLeft1());
		rightStartScale3Cube.addLLL(new RightStartToSwitchLeft1());
		
		AutonRouteChooser centerStartSwitch1Scale2 = new AutonRouteChooser();
		centerStartSwitch1Scale2.addLLL(new CenterStartToSwitchLeft1ScaleLeft2());
		centerStartSwitch1Scale2.addLRL(new CenterStartToSwitchLeft1ScaleRight1());
		centerStartSwitch1Scale2.addRLR(new CenterStartToSwitchRight1ScaleLeft1());
		centerStartSwitch1Scale2.addRRR(new CenterStartToSwitchRight1ScaleRight2());
		
		AutonRouteChooser leftStartScale1Switch1Scale1 = new AutonRouteChooser();
		leftStartScale1Switch1Scale1.addLLL(new StartToScale1Switch1Scale1(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartScale1Switch1Scale1.addRLR(new StartToScale1Switch1Scale1(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartScale1Switch1Scale1.addLRL(new LeftStartToSwitchLeft1ScaleRight1());
		leftStartScale1Switch1Scale1.addRRR(new LeftStartToSwitchRight1());

		AutonRouteChooser leftStartSwitch3 = new AutonRouteChooser();
		leftStartSwitch3.addLLL(new SideStartToSwitch3(new LeftStartToScaleLeft(), new ScaleLeftToSwitchLeftNoIntake(), new SwitchLeftToScaleLeft()));
		leftStartSwitch3.addLRL(new SideStartToSwitch3(new LeftStartToScaleLeft(), new ScaleLeftToSwitchLeftNoIntake(), new SwitchLeftToScaleLeft()));
		leftStartSwitch3.addRLR(new SideStartToSwitch3(new LeftStartToScaleLeft(), new ScaleLeftToSwitchLeftNoIntake(), new SwitchLeftToScaleLeft()));
		leftStartSwitch3.addRRR(new SideStartToSwitch3(new LeftStartToScaleLeft(), new ScaleLeftToSwitchLeftNoIntake(), new SwitchLeftToScaleLeft()));
		
		AutonRouteChooser leftStartTest = new AutonRouteChooser();
		leftStartTest.addLLL(new IntakeTest(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartTest.addRLR(new IntakeTest(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartTest.addLRL(new IntakeTest(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartTest.addRRR(new IntakeTest(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));

		autonTaskChooser.addObject("Center Start Switch 1 Scale 1", centerStartSwitch1Scale1);
		autonTaskChooser.addObject("Center Start Switch 1 Scale 2", centerStartSwitch1Scale2);
		autonTaskChooser.addObject("Right Start Switch 1 Scale 1", rightStartSwitch1Scale1);
		autonTaskChooser.addObject("Left Start Switch 1 Scale 1", leftStartSwitch1Scale1);
		autonTaskChooser.addObject("Center Start Switch 1", centerStartSwitch1);
		autonTaskChooser.addObject("Left Start Scale 3", leftStartScale3Cube);
		autonTaskChooser.addObject("Right Start Scale 3", rightStartScale3Cube);
		autonTaskChooser.addObject("Go Straight Do Nothing", goStraight);
		autonTaskChooser.addObject("Left Start Scale 1 Switch 1 Scale 1", leftStartScale1Switch1Scale1);
		autonTaskChooser.addObject("Left Start Switch 3", leftStartSwitch3);
		autonTaskChooser.addDefault("Test Intake", leftStartTest);
		SmartDashboard.putData("Auton Tasks", autonTaskChooser);
		
		LiveWindow.setEnabled(false);
		LiveWindow.disableAllTelemetry();

		zeroAllSensors();

		drive.setLimeLED(false);
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
    	drive.setShiftState(DriveSpeedShiftState.LO);
    	elevator.setShiftState(Elevator.ElevatorSpeedShiftState.HI);
    	elevator.resetZeroPosition(Elevator.ZERO_POSITION_INCHES);
        zeroAllSensors();

		drive.setLimeLED(false);
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
	    	drive.setShiftState(DriveSpeedShiftState.LO);
		}

		operationMode = operationModeChooser.getSelected();
		
        controlLoop.start();
		ramp.setPullPosition(RampPull.DOWN);
		ramp.setTeleopStartTime();
    	ramp.setOperationMode(operationMode);
    	drive.endGyroCalibration();
    	elevator.setShiftState(Elevator.ElevatorSpeedShiftState.HI);
        zeroAllSensors();

		drive.setLimeLED(false);
		drive.setLimeCameraMode(true);
    	
    	if (operationMode != OperationMode.COMPETITION) {
    		Scheduler.getInstance().add(new ElevatorAutoZero(false));
    	}
    	else {
            elevator.setPositionPID(elevator.getPositionInches());
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
    	robotState.updateStatus(operationMode);
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
