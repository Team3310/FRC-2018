package org.usfirst.frc.team3310.robot.commands.auton;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.DriveStraightMPIntakeOff;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.FlipperFlip;
import org.usfirst.frc.team3310.robot.commands.IntakeCubeAndLiftAbortDrive;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeed;
import org.usfirst.frc.team3310.robot.commands.RunAfterMarker;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Flipper.FlipperSide;
import org.usfirst.frc.team3310.robot.subsystems.Flipper.FlipperState;
import org.usfirst.frc.team3310.robot.subsystems.Intake;
import org.usfirst.frc.team3310.utility.Path;
import org.usfirst.frc.team3310.utility.Path.Waypoint;
import org.usfirst.frc.team3310.utility.Translation2d;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class RightSideScaleAuton extends CommandGroup {

    public RightSideScaleAuton() {
    	
    	//  SAVE!!!   Start right to scale
 		List<Waypoint> waypoints = new ArrayList<>();
//		waypoints.add(new Waypoint(new Translation2d(0, 0), 50.0));
//		waypoints.add(new Waypoint(new Translation2d(0, -80), 50.0, "launchFlipper"));
//		waypoints.add(new Waypoint(new Translation2d(0, -170), 50.0, 30.0));
//		waypoints.add(new Waypoint(new Translation2d(-21, -220), 50.0, 50.0));
//		waypoints.add(new Waypoint(new Translation2d(-21, -230), 50.0));
//		waypoints.add(new Waypoint(new Translation2d(-21, -250), 10.0));
		
		//  Start right to scale
		waypoints.add(new Waypoint(new Translation2d(0, 0), 30.0));
		waypoints.add(new Waypoint(new Translation2d(0, -80), 30.0, "launchFlipper"));
		waypoints.add(new Waypoint(new Translation2d(0, -170), 50.0, 30.0));
		waypoints.add(new Waypoint(new Translation2d(-15, -210), 50.0, 50.0));
		waypoints.add(new Waypoint(new Translation2d(-51, -218), 40.0));
		waypoints.add(new Waypoint(new Translation2d(-15, -224), 10.0));
		waypoints.add(new Waypoint(new Translation2d(-15, -225), 10.0));
		
    	addParallel(new RunAfterMarker("launchFlipper", 3.0, new FlipperFlip(FlipperSide.RIGHT, FlipperState.DEPLOYED)));
    	addSequential(new DrivePathAdaptivePursuit(new Path(waypoints), true));
		addSequential(new FlipperFlip(FlipperSide.RIGHT, FlipperState.RETRACTED));

		// Forward for first cube
//		List<Waypoint> waypoints2 = new ArrayList<>();
//		waypoints2.add(new Waypoint(new Translation2d(0, 0), 50.0));
//		waypoints2.add(new Waypoint(new Translation2d(0, -0), 50.0));

//		addSequential(new IntakeCubeAndLiftAbortDrive());
		addSequential(new IntakeSetSpeed(Intake.INTAKE_LOAD_SPEED));
		addSequential(new DriveStraightMPIntakeOff(50, Drive.MP_AUTON_MAX_LO_GEAR_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, true, 0));
		addParallel(new ElevatorSetPositionMP(Elevator.MAX_POSITION_INCHES));
		addSequential(new DriveStraightMP(-70, Drive.MP_AUTON_MAX_LO_GEAR_STRAIGHT_VELOCITY_INCHES_PER_SEC, true, true, 0));
		addSequential(new IntakeSetSpeed(Intake.INTAKE_LOAD_SPEED));
		addSequential(new WaitCommand(1));
		addSequential(new IntakeSetSpeed(0));
//		addSequential(new DriveGyroReset());
//		addSequential(new DrivePathAdaptivePursuit(new Path(waypoints2), false));
     }
}
