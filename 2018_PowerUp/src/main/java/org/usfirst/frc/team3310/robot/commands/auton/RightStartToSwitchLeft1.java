package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.RightStartToScaleLeft;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeft;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightStartToSwitchLeft1 extends CommandGroup {

    public RightStartToSwitchLeft1() {
    
    	addSequential(new SideStartToSwitch1(new RightStartToScaleLeft(), new ScaleLeftToSwitchLeft()));

    }
}
