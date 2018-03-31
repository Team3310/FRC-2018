package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.paths.auton.LeftStartToSwitchRight;
import org.usfirst.frc.team3310.paths.auton.PyramidToSwitchRight;
import org.usfirst.frc.team3310.paths.auton.SwitchRightToCenterStart;
import org.usfirst.frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import org.usfirst.frc.team3310.robot.commands.DriveResetPoseFromPath;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetPositionMP;
import org.usfirst.frc.team3310.robot.commands.ElevatorSetZero;
import org.usfirst.frc.team3310.robot.commands.ParallelDelay;
import org.usfirst.frc.team3310.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftStartToSwitchRight2 extends CommandGroup {

    public LeftStartToSwitchRight2(PathContainer leftStartToCenterStart) {
    	addSequential(new ElevatorSetZero(0));
    	
        addSequential(new DriveResetPoseFromPath(leftStartToCenterStart, true));
    	addSequential(new DrivePathAdaptivePursuit(leftStartToCenterStart));

    	addSequential(new CenterStartToSwitch2(new LeftStartToSwitchRight(), new SwitchRightToCenterStart(), new PyramidToSwitchRight(),false));
    }
}
