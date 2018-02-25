package org.usfirst.frc.team3310.robot.commands.auton;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.utility.Path;
import org.usfirst.frc.team3310.utility.Translation2d;
import org.usfirst.frc.team3310.utility.Path.Waypoint;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightSideScaleAuton extends CommandGroup {

    public RightSideScaleAuton() {
 		List<Waypoint> waypoints = new ArrayList<>();
		waypoints.add(new Waypoint(new Translation2d(0, 0), 50.0));
		waypoints.add(new Waypoint(new Translation2d(0, -150), 50.0, 30.0));
		waypoints.add(new Waypoint(new Translation2d(-20, -200), 50.0, 50.0));
		waypoints.add(new Waypoint(new Translation2d(-20, -260), 50.0));

    	addSequential(new DrivePathAdaptivePursuit(new Path(waypoints), true));
 //   	addSequential(command);
    }
}
