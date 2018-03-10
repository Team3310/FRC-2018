package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.CenterStartToScaleRight;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight2;
import org.usfirst.frc.team3310.paths.auton.SwitchRight2ToScaleRight;
import org.usfirst.frc.team3310.paths.auton.SwitchRightToScaleRight;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CenterStartToSwitchRight1ScaleRight2 extends CommandGroup {

    public CenterStartToSwitchRight1ScaleRight2() {
    
    	addSequential(new CenterStartToSwitch1Scale1(new CenterStartToScaleRight(), new ScaleRightToSwitchRight(), new SwitchRightToScaleRight()));
    	addSequential(new ScaleToSwitchCube2(new ScaleRightToSwitchRight2(), new SwitchRight2ToScaleRight()));
    }
}
