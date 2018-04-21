/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3310.robot;

import java.util.HashMap;

import org.usfirst.frc.team3310.paths.auton.LeftStartToScaleLeft;
import org.usfirst.frc.team3310.paths.auton.LeftStartToScaleLeftV2;
import org.usfirst.frc.team3310.paths.auton.LeftStartToScaleRightAPRV3;
import org.usfirst.frc.team3310.paths.auton.LeftStartToScaleRightSave;
import org.usfirst.frc.team3310.paths.auton.LeftStartToSwitchRightV3;
import org.usfirst.frc.team3310.paths.auton.LeftSwitch2ndCubeV2;
import org.usfirst.frc.team3310.paths.auton.PyramidToSwitchLeftV2;
import org.usfirst.frc.team3310.paths.auton.PyramidToSwitchRight;
import org.usfirst.frc.team3310.paths.auton.RightStartToScaleLeftAPRV3;
import org.usfirst.frc.team3310.paths.auton.RightStartToScaleLeftSave;
import org.usfirst.frc.team3310.paths.auton.RightStartToScaleRight;
import org.usfirst.frc.team3310.paths.auton.RightStartToScaleRightV2;
import org.usfirst.frc.team3310.paths.auton.RightStartToSwitchLeftV3;
import org.usfirst.frc.team3310.paths.auton.RightSwitch2ndCubeV2;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToOuttaHere;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeft;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeft2;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeftNoIntake;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeftV3;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToOuttaHere;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight23Cube;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight3Cube;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRightNoIntake;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRightV3;
import org.usfirst.frc.team3310.paths.auton.SwitchLeft2ToScaleLeft;
import org.usfirst.frc.team3310.paths.auton.SwitchLeftToCenterStartV2;
import org.usfirst.frc.team3310.paths.auton.SwitchLeftToScaleLeft;
import org.usfirst.frc.team3310.paths.auton.SwitchRight2ToScaleRight3Cube;
import org.usfirst.frc.team3310.paths.auton.SwitchRightToCenterStart;
import org.usfirst.frc.team3310.paths.auton.SwitchRightToScaleRight;
import org.usfirst.frc.team3310.paths.auton.SwitchRightToScaleRight3Cube;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorAutoZero;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchLeft1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchLeft2;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchRight1;
import org.usfirst.frc.team3310.robot.commands.auton.CenterStartToSwitchRight2;
import org.usfirst.frc.team3310.robot.commands.auton.LeftStartToScaleLeft1SwitchRight1V2;
import org.usfirst.frc.team3310.robot.commands.auton.LeftStartToSwitchLeft1ScaleRight1;
import org.usfirst.frc.team3310.robot.commands.auton.RightStartToScaleRight1SwitchLeft1V2;
import org.usfirst.frc.team3310.robot.commands.auton.RightStartToSwitchRight1ScaleLeft1;
import org.usfirst.frc.team3310.robot.commands.auton.SideStartToOppositeScale1Switch1;
import org.usfirst.frc.team3310.robot.commands.auton.SideStartToOppositeScale2;
import org.usfirst.frc.team3310.robot.commands.auton.SideStartToScale1APR;
import org.usfirst.frc.team3310.robot.commands.auton.SideStartToSwitch2;
import org.usfirst.frc.team3310.robot.commands.auton.SideStartToSwitch3;
import org.usfirst.frc.team3310.robot.commands.auton.StartToScale1OuttaHere;
import org.usfirst.frc.team3310.robot.commands.auton.StartToScale1Switch1Scale1;
import org.usfirst.frc.team3310.robot.commands.auton.StartToScale3;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Drive.DriveSpeedShiftState;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Forks;
import org.usfirst.frc.team3310.robot.subsystems.Forks.ForksLockState;
import org.usfirst.frc.team3310.robot.subsystems.Intake;
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
	public static final Forks forks = Forks.getInstance();
	
	// Control looper
	public static final Looper controlLoop = new Looper();
	
	// Choosers
	private SendableChooser<OperationMode> operationModeChooser;
	private SendableChooser<AutonRouteChooser> autonTaskChooser;
    private Command autonomousCommand;

	public static enum OperationMode { TEST, PRACTICE, COMPETITION };
	public static OperationMode operationMode = OperationMode.COMPETITION;

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
		
		AutonRouteChooser goStraightForward = new AutonRouteChooser();
		goStraightForward.addLLL(new DriveStraightMP(150.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));
		goStraightForward.addLRL(new DriveStraightMP(150.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));
		goStraightForward.addRLR(new DriveStraightMP(150.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));
		goStraightForward.addRRR(new DriveStraightMP(150.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));

		AutonRouteChooser goStraightBackward = new AutonRouteChooser();
		goStraightBackward.addLLL(new DriveStraightMP(-150.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));
		goStraightBackward.addLRL(new DriveStraightMP(-150.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));
		goStraightBackward.addRLR(new DriveStraightMP(-150.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));
		goStraightBackward.addRRR(new DriveStraightMP(-150.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));

		AutonRouteChooser centerStartSwitch1 = new AutonRouteChooser();
		centerStartSwitch1.addLLL(new CenterStartToSwitchLeft1());
		centerStartSwitch1.addLRL(new CenterStartToSwitchLeft1());
		centerStartSwitch1.addRLR(new CenterStartToSwitchRight1());
		centerStartSwitch1.addRRR(new CenterStartToSwitchRight1());
		
		AutonRouteChooser centerStartSwitch2 = new AutonRouteChooser();
		centerStartSwitch2.addLLL(new CenterStartToSwitchLeft2());
		centerStartSwitch2.addLRL(new CenterStartToSwitchLeft2());
		centerStartSwitch2.addRLR(new CenterStartToSwitchRight2());
		centerStartSwitch2.addRRR(new CenterStartToSwitchRight2());

		AutonRouteChooser leftStartScale1Switch1Scale1 = new AutonRouteChooser();
		leftStartScale1Switch1Scale1.addLLL(new StartToScale1Switch1Scale1(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new LeftSwitch2ndCubeV2(), new SwitchLeft2ToScaleLeft(), false));
		leftStartScale1Switch1Scale1.addRLR(new LeftStartToScaleLeft1SwitchRight1V2());
		leftStartScale1Switch1Scale1.addLRL(new LeftStartToSwitchLeft1ScaleRight1());
		leftStartScale1Switch1Scale1.addRRR(new SideStartToOppositeScale1Switch1(new LeftStartToScaleRightSave(), new ScaleRightToSwitchRight(), false));

		AutonRouteChooser leftStartScale2or3 = new AutonRouteChooser();
		leftStartScale2or3.addLLL(new StartToScale3(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartScale2or3.addRLR(new StartToScale3(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartScale2or3.addLRL(new SideStartToOppositeScale2(new LeftStartToScaleRightSave(), new ScaleRightToSwitchRightV3(), new SwitchRightToScaleRight(), false));
		leftStartScale2or3.addRRR(new SideStartToOppositeScale2(new LeftStartToScaleRightSave(), new ScaleRightToSwitchRightV3(), new SwitchRightToScaleRight(), false));

		AutonRouteChooser leftStartScale3APR = new AutonRouteChooser();
		leftStartScale3APR.addLLL(new StartToScale3(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartScale3APR.addRLR(new StartToScale3(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartScale3APR.addLRL(new SideStartToScale1APR(new LeftStartToScaleRightAPRV3(), false));
		leftStartScale3APR.addRRR(new SideStartToScale1APR(new LeftStartToScaleRightAPRV3(), false));

		AutonRouteChooser leftStartScale3Switch3APR = new AutonRouteChooser();
		leftStartScale3Switch3APR.addLLL(new StartToScale3(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartScale3Switch3APR.addRLR(new StartToScale3(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartScale3Switch3APR.addLRL(new SideStartToSwitch3(new LeftStartToScaleLeft(), new ScaleLeftToSwitchLeftNoIntake(), new LeftSwitch2ndCubeV2(), false));
		leftStartScale3Switch3APR.addRRR(new SideStartToScale1APR(new LeftStartToScaleRightAPRV3(), false));

		AutonRouteChooser leftStartScale3Switch2or3 = new AutonRouteChooser();
		leftStartScale3Switch2or3.addLLL(new StartToScale3(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartScale3Switch2or3.addRLR(new StartToScale3(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft(), new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
		leftStartScale3Switch2or3.addLRL(new SideStartToSwitch3(new LeftStartToScaleLeft(), new ScaleLeftToSwitchLeftNoIntake(), new LeftSwitch2ndCubeV2(), false));
		leftStartScale3Switch2or3.addRRR(new SideStartToSwitch2(new LeftStartToSwitchRightV3(),new SwitchRightToCenterStart(), new PyramidToSwitchRight(), false));

		AutonRouteChooser leftStartScale1OuttaHereAPR = new AutonRouteChooser();
		leftStartScale1OuttaHereAPR.addLLL(new StartToScale1OuttaHere(new LeftStartToScaleLeftV2(), new ScaleLeftToOuttaHere()));
		leftStartScale1OuttaHereAPR.addRLR(new StartToScale1OuttaHere(new LeftStartToScaleLeftV2(), new ScaleLeftToOuttaHere()));
		leftStartScale1OuttaHereAPR.addLRL(new SideStartToScale1APR(new LeftStartToScaleRightAPRV3(), false));
		leftStartScale1OuttaHereAPR.addRRR(new SideStartToScale1APR(new LeftStartToScaleRightAPRV3(), false));

		AutonRouteChooser leftStartScale1OuttaHere3Switch = new AutonRouteChooser();
		leftStartScale1OuttaHere3Switch.addLLL(new StartToScale1OuttaHere(new LeftStartToScaleLeftV2(), new ScaleLeftToOuttaHere()));
		leftStartScale1OuttaHere3Switch.addRLR(new StartToScale1OuttaHere(new LeftStartToScaleLeftV2(), new ScaleLeftToOuttaHere()));
		leftStartScale1OuttaHere3Switch.addLRL(new SideStartToSwitch3(new LeftStartToScaleLeft(), new ScaleLeftToSwitchLeftNoIntake(), new LeftSwitch2ndCubeV2(), false));
		leftStartScale1OuttaHere3Switch.addRRR(new SideStartToScale1APR(new LeftStartToScaleRightAPRV3(), false));

		AutonRouteChooser leftStartScale1OuttaHereAPR3Switch = new AutonRouteChooser();
		leftStartScale1OuttaHereAPR3Switch.addLLL(new StartToScale1OuttaHere(new LeftStartToScaleLeftV2(), new ScaleLeftToOuttaHere()));
		leftStartScale1OuttaHereAPR3Switch.addRLR(new StartToScale1OuttaHere(new LeftStartToScaleLeftV2(), new ScaleLeftToOuttaHere()));
		leftStartScale1OuttaHereAPR3Switch.addLRL(new SideStartToSwitch3(new LeftStartToScaleLeft(), new ScaleLeftToSwitchLeftNoIntake(), new LeftSwitch2ndCubeV2(), false));
		leftStartScale1OuttaHereAPR3Switch.addRRR(new DriveStraightMP(-150.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));

		AutonRouteChooser rightStartScale1Switch1Scale1 = new AutonRouteChooser();
		rightStartScale1Switch1Scale1.addLLL(new SideStartToOppositeScale1Switch1(new RightStartToScaleLeftSave(), new ScaleLeftToSwitchLeftV3(), true));
		rightStartScale1Switch1Scale1.addRLR(new RightStartToSwitchRight1ScaleLeft1());
		rightStartScale1Switch1Scale1.addLRL(new RightStartToScaleRight1SwitchLeft1V2());
		rightStartScale1Switch1Scale1.addRRR(new StartToScale1Switch1Scale1(new RightStartToScaleRightV2(), new ScaleRightToSwitchRight3Cube(), new RightSwitch2ndCubeV2(), new SwitchRight2ToScaleRight3Cube(), true));

		AutonRouteChooser rightStartScale2or3 = new AutonRouteChooser();
		rightStartScale2or3.addLLL(new SideStartToOppositeScale2(new RightStartToScaleLeftSave(), new ScaleLeftToSwitchLeftV3(), new SwitchLeftToScaleLeft(), true));
		rightStartScale2or3.addRLR(new SideStartToOppositeScale2(new RightStartToScaleLeftSave(), new ScaleLeftToSwitchLeftV3(), new SwitchLeftToScaleLeft(), true));
		rightStartScale2or3.addLRL(new StartToScale3(new RightStartToScaleRightV2(), new ScaleRightToSwitchRight3Cube(), new SwitchRightToScaleRight3Cube(), new ScaleRightToSwitchRight23Cube(), new SwitchRight2ToScaleRight3Cube()));
		rightStartScale2or3.addRRR(new StartToScale3(new RightStartToScaleRightV2(), new ScaleRightToSwitchRight3Cube(), new SwitchRightToScaleRight3Cube(), new ScaleRightToSwitchRight23Cube(), new SwitchRight2ToScaleRight3Cube()));

		AutonRouteChooser rightStartScale3APR = new AutonRouteChooser();
		rightStartScale3APR.addLLL(new SideStartToScale1APR(new RightStartToScaleLeftAPRV3(), true));
		rightStartScale3APR.addRLR(new SideStartToScale1APR(new RightStartToScaleLeftAPRV3(), true));
		rightStartScale3APR.addLRL(new StartToScale3(new RightStartToScaleRightV2(), new ScaleRightToSwitchRight3Cube(), new SwitchRightToScaleRight3Cube(), new ScaleRightToSwitchRight23Cube(), new SwitchRight2ToScaleRight3Cube()));
		rightStartScale3APR.addRRR(new StartToScale3(new RightStartToScaleRightV2(), new ScaleRightToSwitchRight3Cube(), new SwitchRightToScaleRight3Cube(), new ScaleRightToSwitchRight23Cube(), new SwitchRight2ToScaleRight3Cube()));

		AutonRouteChooser rightStartScale3Switch3APR = new AutonRouteChooser();
		rightStartScale3Switch3APR.addLLL(new SideStartToScale1APR(new RightStartToScaleLeftAPRV3(), true));
		rightStartScale3Switch3APR.addRLR(new SideStartToSwitch3(new RightStartToScaleRight(), new ScaleRightToSwitchRightNoIntake(), new RightSwitch2ndCubeV2(), true));
		rightStartScale3Switch3APR.addLRL(new StartToScale3(new RightStartToScaleRightV2(), new ScaleRightToSwitchRight3Cube(), new SwitchRightToScaleRight3Cube(), new ScaleRightToSwitchRight23Cube(), new SwitchRight2ToScaleRight3Cube()));
		rightStartScale3Switch3APR.addRRR(new StartToScale3(new RightStartToScaleRightV2(), new ScaleRightToSwitchRight3Cube(), new SwitchRightToScaleRight3Cube(), new ScaleRightToSwitchRight23Cube(), new SwitchRight2ToScaleRight3Cube()));

		AutonRouteChooser rightStartScale3Switch2or3 = new AutonRouteChooser();
		rightStartScale3Switch2or3.addLLL(new SideStartToSwitch2(new RightStartToSwitchLeftV3(),new SwitchLeftToCenterStartV2(), new PyramidToSwitchLeftV2(), true));
		rightStartScale3Switch2or3.addRLR(new SideStartToSwitch3(new RightStartToScaleRight(), new ScaleRightToSwitchRightNoIntake(), new RightSwitch2ndCubeV2(), true));
		rightStartScale3Switch2or3.addLRL(new StartToScale3(new RightStartToScaleRightV2(), new ScaleRightToSwitchRight3Cube(), new SwitchRightToScaleRight3Cube(), new ScaleRightToSwitchRight23Cube(), new SwitchRight2ToScaleRight3Cube()));
		rightStartScale3Switch2or3.addRRR(new StartToScale3(new RightStartToScaleRightV2(), new ScaleRightToSwitchRight3Cube(), new SwitchRightToScaleRight3Cube(), new ScaleRightToSwitchRight23Cube(), new SwitchRight2ToScaleRight3Cube()));

		AutonRouteChooser rightStartScale1OuttaHereAPR = new AutonRouteChooser();
		rightStartScale1OuttaHereAPR.addLLL(new SideStartToScale1APR(new RightStartToScaleLeftAPRV3(), true));
		rightStartScale1OuttaHereAPR.addRLR(new SideStartToScale1APR(new RightStartToScaleLeftAPRV3(), true));
		rightStartScale1OuttaHereAPR.addLRL(new StartToScale1OuttaHere(new RightStartToScaleRightV2(), new ScaleRightToOuttaHere()));
		rightStartScale1OuttaHereAPR.addRRR(new StartToScale1OuttaHere(new RightStartToScaleRightV2(), new ScaleRightToOuttaHere()));

		AutonRouteChooser rightStartScale1OuttaHere3Switch = new AutonRouteChooser();
		rightStartScale1OuttaHere3Switch.addLLL(new SideStartToScale1APR(new RightStartToScaleLeftAPRV3(), true));
		rightStartScale1OuttaHere3Switch.addRLR(new SideStartToSwitch3(new RightStartToScaleRight(), new ScaleRightToSwitchRightNoIntake(), new RightSwitch2ndCubeV2(), true));
		rightStartScale1OuttaHere3Switch.addLRL(new StartToScale1OuttaHere(new RightStartToScaleRightV2(), new ScaleRightToOuttaHere()));
		rightStartScale1OuttaHere3Switch.addRRR(new StartToScale1OuttaHere(new RightStartToScaleRightV2(), new ScaleRightToOuttaHere()));

		AutonRouteChooser rightStartScale1OuttaHereAPR3Switch = new AutonRouteChooser();
		rightStartScale1OuttaHereAPR3Switch.addLLL(new DriveStraightMP(-150.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));
		rightStartScale1OuttaHereAPR3Switch.addRLR(new SideStartToSwitch3(new RightStartToScaleRight(), new ScaleRightToSwitchRightNoIntake(), new RightSwitch2ndCubeV2(), true));
		rightStartScale1OuttaHereAPR3Switch.addLRL(new StartToScale1OuttaHere(new RightStartToScaleRightV2(), new ScaleRightToOuttaHere()));
		rightStartScale1OuttaHereAPR3Switch.addRRR(new StartToScale1OuttaHere(new RightStartToScaleRightV2(), new ScaleRightToOuttaHere()));

		// Extra routines		
//		rightStartScale3Cube.addLLL(new SideStartToScale1APRV2(new RightStartToScaleLeftAPRV2(), true));
//		rightStartScale3Cube.addLLL(new StartToScale1Pyramid1(new RightStartToScaleRightV2(), new ScaleRightToPyramid(), new PyramidToScaleRight(), new ScaleRightToSwitchRight23Cube(), new SwitchRight2ToScaleRight3Cube()));

		autonTaskChooser.addObject("C0 Go Straight Backward Do Nothing", goStraightBackward);
		autonTaskChooser.addObject("C1 Go Straight Forward Do Nothing", goStraightForward);
		autonTaskChooser.addObject("C2 Center Start Switch 1", centerStartSwitch1);
		autonTaskChooser.addDefault("C3 Center Start Switch 2", centerStartSwitch2);
		
		autonTaskChooser.addObject("L1 Left Start Scale 1 Switch 1 Scale 1", leftStartScale1Switch1Scale1);
		autonTaskChooser.addObject("L2 Left Start Scale 3 Same Scale 2 Opposite", leftStartScale2or3);
		autonTaskChooser.addObject("L3 Left Start Scale 3 Same APR Opposite", leftStartScale3APR);
		autonTaskChooser.addObject("L4 Left Start Scale 3 Same Switch 3 or APR Opposite", leftStartScale3Switch3APR);
		autonTaskChooser.addObject("L5 Left Start Scale 3 Same Switch 2 or 3 Opposite", leftStartScale3Switch2or3);
		autonTaskChooser.addObject("L6 Left Start Scale 1 Outta Here Same APR Opposite", leftStartScale1OuttaHereAPR);
		autonTaskChooser.addObject("L7 Left Start Scale 1 Outta Here Same 3 Switch APR Opposite", leftStartScale1OuttaHere3Switch);

		autonTaskChooser.addObject("R1 Right Start Scale 1 Switch 1 Scale 1", rightStartScale1Switch1Scale1);
		autonTaskChooser.addObject("R2 Right Start Scale 3 Same Scale 2 Opposite", rightStartScale2or3);
		autonTaskChooser.addObject("R3 Right Start Scale 3 Same APR Opposite", rightStartScale3APR);
		autonTaskChooser.addObject("R4 Right Start Scale 3 Same Switch 3 or APR Opposite", rightStartScale3Switch3APR);
		autonTaskChooser.addObject("R5 Right Start Scale 3 Same Switch 2 or 3 Opposite", rightStartScale3Switch2or3);
		autonTaskChooser.addObject("R6 Right Start Scale 1 Outta Here Same APR Opposite", rightStartScale1OuttaHereAPR);
		autonTaskChooser.addObject("R7 Right Start Scale 1 Outta Here Same 3 Switch APR Oppisite", rightStartScale1OuttaHere3Switch);

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
		}

		operationMode = operationModeChooser.getSelected();
		
        controlLoop.start();
		forks.setLockState(ForksLockState.STOWED);
    	drive.setShiftState(DriveSpeedShiftState.LO);
    	drive.endGyroCalibration();
    	elevator.setShiftState(Elevator.ElevatorSpeedShiftState.HI);
        zeroAllSensors();

		drive.setLimeLED(false);
		drive.setLimeCameraMode(false);
    	
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
    	forks.updateStatus(operationMode);
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
