package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.RightStartToScaleRight;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight;
import org.usfirst.frc.team3310.paths.auton.SwitchRightToScaleRight;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightStartToSwitchRight1ScaleRight1 extends CommandGroup {

    public RightStartToSwitchRight1ScaleRight1() {
    
    	addSequential(new SideStartToSwitch1Scale1(new RightStartToScaleRight(), new ScaleRightToSwitchRight(), new SwitchRightToScaleRight()));

    }
}
