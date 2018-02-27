package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.robot.commands.DriveStraightMP;
import org.usfirst.frc.team3310.robot.commands.DriveStraightUntilCube;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.FlipperFlip;
import org.usfirst.frc.team3310.robot.commands.IntakeSetSpeed;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Flipper.FlipperSide;
import org.usfirst.frc.team3310.robot.subsystems.Flipper.FlipperState;
import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class RightSideScaleAuton extends CommandGroup {

    public RightSideScaleAuton() {
    	
    	//  SAVE!!!   Start right to scale
// 		List<Waypoint> waypoints = new ArrayList<>();
//		waypoints.add(new Waypoint(new Translation2d(0, 0), 50.0));
//		waypoints.add(new Waypoint(new Translation2d(0, -80), 50.0, "launchFlipper"));
//		waypoints.add(new Waypoint(new Translation2d(0, -170), 50.0, 30.0));
//		waypoints.add(new Waypoint(new Translation2d(-21, -220), 50.0, 50.0));
//		waypoints.add(new Waypoint(new Translation2d(-21, -230), 50.0));
//		waypoints.add(new Waypoint(new Translation2d(-21, -250), 10.0));
		
		//  Start right to scale
//		waypoints.add(new Waypoint(new Translation2d(0, 0), 30.0));
//		waypoints.add(new Waypoint(new Translation2d(0, -70), 40.0, "launchFlipper"));
//		waypoints.add(new Waypoint(new Translation2d(0, -170), 40.0, 30.0));
//		waypoints.add(new Waypoint(new Translation2d(-25, -220), 40.0, 50.0));
//		waypoints.add(new Waypoint(new Translation2d(-25, -226), 40.0));
//		waypoints.add(new Waypoint(new Translation2d(-23, -235), 15.0));
		
//    	addParallel(new RunAfterMarker("launchFlipper", 3.0, new FlipperFlip(FlipperSide.RIGHT, FlipperState.DEPLOYED)));
//    	addSequential(new DrivePathAdaptivePursuit(new Path(waypoints, true)));
		addSequential(new FlipperFlip(FlipperSide.RIGHT, FlipperState.RETRACTED));

		addSequential(new DriveStraightUntilCube());
		addParallel(new ElevatorSetPositionMP(Elevator.MAX_POSITION_INCHES));
		addSequential(new DriveStraightMP(-70, 50, true, true, 0));
		addSequential(new IntakeSetSpeed(Intake.INTAKE_REAR_EJECT_SPEED));
		addSequential(new WaitCommand(1));
		addSequential(new IntakeSetSpeed(0));
//		addSequential(new DriveGyroReset());
//		addSequential(new DrivePathAdaptivePursuit(new Path(waypoints2), false));
     }
}
