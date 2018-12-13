package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.CenterStartToScaleRight;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight;
import org.usfirst.frc.team3310.paths.auton.SwitchRightToScaleRight;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CenterStartToSwitchRight1ScaleRight1 extends CommandGroup {

    public CenterStartToSwitchRight1ScaleRight1() {
    
    	addSequential(new CenterStartToSwitch1Scale1(new CenterStartToScaleRight(), new ScaleRightToSwitchRight(), new SwitchRightToScaleRight(), true));

    }
}
