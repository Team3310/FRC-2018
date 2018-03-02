/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3310.robot;

import org.usfirst.frc.team3310.robot.commands.ElevatorAutoZero;
import org.usfirst.frc.team3310.robot.commands.auton.RightSideScaleAuton;
import org.usfirst.frc.team3310.robot.commands.auton.TestAuton;
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
import edu.wpi.first.networktables.NetworkTableInstance;
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
	private SendableChooser<Command> autonTaskChooser;
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

		autonTaskChooser = new SendableChooser<Command>();
		autonTaskChooser.addDefault("Test", new TestAuton());
		autonTaskChooser.addObject("Right Side Scale", new RightSideScaleAuton());
		SmartDashboard.putData("Auton Tasks", autonTaskChooser);
		
		LiveWindow.setEnabled(false);
		LiveWindow.disableAllTelemetry();

		zeroAllSensors();

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); 
	}  
	
	// Called every loop for all modes
	public void robotPeriodic() {
		updateStatus();
	}

	@Override
	public void disabledInit() {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); 
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
        zeroAllSensors();
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = autonTaskChooser.getSelected();

    	controlLoop.start();
    	drive.setIsRed(getAlliance().equals(Alliance.Red));
    	elevator.setShiftState(Elevator.ElevatorSpeedShiftState.HI);
    	elevator.resetZeroPosition(Elevator.ZERO_POSITION_INCHES);
        zeroAllSensors();

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); 

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

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); 
    	
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

}
