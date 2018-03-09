package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.RightStartToScaleRight;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight;
import org.usfirst.frc.team3310.paths.auton.SwitchRightToScaleLeft;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightStartToSwitchRight1ScaleLeft1 extends CommandGroup {

    public RightStartToSwitchRight1ScaleLeft1() {
    
    	addSequential(new SideStartToSwitch1Scale1(new RightStartToScaleRight(), new ScaleRightToSwitchRight(), new SwitchRightToScaleLeft()));

    }
}
