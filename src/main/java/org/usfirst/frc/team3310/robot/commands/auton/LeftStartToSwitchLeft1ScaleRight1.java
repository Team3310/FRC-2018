package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.LeftStartToScaleLeft;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeftNoIntake;
import org.usfirst.frc.team3310.paths.auton.SwitchLeftToScaleRight;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftStartToSwitchLeft1ScaleRight1 extends CommandGroup {

    public LeftStartToSwitchLeft1ScaleRight1() {
    
    	addSequential(new SideStartToSwitch1Scale1(new LeftStartToScaleLeft(), new ScaleLeftToSwitchLeftNoIntake(), new SwitchLeftToScaleRight()));

    }
}
