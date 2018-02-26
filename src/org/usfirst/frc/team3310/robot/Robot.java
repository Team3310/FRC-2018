/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3310.robot;

import org.usfirst.frc.team3310.robot.commands.ElevatorAutoZero;
import org.usfirst.frc.team3310.robot.commands.auton.RightSideScaleAuton;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Flipper;
import org.usfirst.frc.team3310.robot.subsystems.Intake;
import org.usfirst.frc.team3310.robot.subsystems.Ramp;
import org.usfirst.frc.team3310.utility.ControlLooper;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
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
	public static final long periodMS = 10;
	public static final ControlLooper controlLoop = new ControlLooper("Main control loop", periodMS);
	
	// Choosers
	private SendableChooser<OperationMode> operationModeChooser;
	private SendableChooser<Command> autonTaskChooser;
    private Command autonomousCommand;

	public static enum OperationMode { TEST, PRACTICE, COMPETITION };
	public static OperationMode operationMode = OperationMode.TEST;

	// PDP
	public static final PowerDistributionPanel pdp = new PowerDistributionPanel();

	@Override
	public void robotInit() {
		setPeriod(periodMS/1000.0);
		
		oi = OI.getInstance();
		
    	controlLoop.addLoopable(drive);
    	controlLoop.addLoopable(elevator);
    	controlLoop.start();
 
    	// Update default at competition!!!
    	operationModeChooser = new SendableChooser<OperationMode>();
	    operationModeChooser.addObject("Practice", OperationMode.PRACTICE);
	    operationModeChooser.addObject("Competition", OperationMode.COMPETITION);
	    operationModeChooser.addDefault("Test", OperationMode.TEST);
		SmartDashboard.putData("Operation Mode", operationModeChooser);

		autonTaskChooser = new SendableChooser<Command>();
		autonTaskChooser.addDefault("Right Side Scale", new RightSideScaleAuton());
		SmartDashboard.putData("Auton Tasks", autonTaskChooser);

		updateStatus();
	}  
	
	// Called every loop for all modes
	public void robotPeriodic() {
	}

	@Override
	public void disabledInit() {
		updateStatus();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		updateStatus();
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = autonTaskChooser.getSelected();

    	controlLoop.start();
    	drive.endGyroCalibration();
    	drive.resetEncoders();
    	drive.resetGyro();
    	drive.setIsRed(getAlliance().equals(Alliance.Red));
    	elevator.setShiftState(Elevator.SpeedShiftState.HI);
    	elevator.resetZeroPosition(Elevator.ZERO_POSITION_INCHES);

		if (autonomousCommand != null) {
			autonomousCommand.start();
		}

		updateStatus();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();

		updateStatus();
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		operationMode = operationModeChooser.getSelected();
		
        controlLoop.start();
    	drive.resetEncoders();
    	drive.endGyroCalibration();
    	elevator.setShiftState(Elevator.SpeedShiftState.HI);
    	
    	if (operationMode != OperationMode.COMPETITION) {
    		Scheduler.getInstance().add(new ElevatorAutoZero(false));
    	}
 
    	updateStatus();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		updateStatus();
	}
	
    public Alliance getAlliance() {
    	return m_ds.getAlliance();
    }
    
    public void updateStatus() {
    	drive.updateStatus(operationMode);
    	intake.updateStatus(operationMode);
    	elevator.updateStatus(operationMode);
    	flipper.updateStatus(operationMode);
    	ramp.updateStatus(operationMode);
    }

}
