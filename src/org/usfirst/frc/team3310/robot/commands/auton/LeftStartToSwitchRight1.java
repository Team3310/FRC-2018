package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.LeftStartToScaleRight;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftStartToSwitchRight1 extends CommandGroup {

    public LeftStartToSwitchRight1() {
    
    	addSequential(new SideStartToSwitch1(new LeftStartToScaleRight(), new ScaleRightToSwitchRight()));

    }
}
