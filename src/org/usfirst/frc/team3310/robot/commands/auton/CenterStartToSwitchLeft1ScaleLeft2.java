package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.CenterStartToScaleLeft;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeft;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeft2;
import org.usfirst.frc.team3310.paths.auton.SwitchLeft2ToScaleLeft;
import org.usfirst.frc.team3310.paths.auton.SwitchLeftToScaleLeft;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CenterStartToSwitchLeft1ScaleLeft2 extends CommandGroup {

    public CenterStartToSwitchLeft1ScaleLeft2() {
    
    	addSequential(new CenterStartToSwitch1Scale1(new CenterStartToScaleLeft(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleLeft()));
    	addSequential(new ScaleToSwitchCube2(new ScaleLeftToSwitchLeft2(), new SwitchLeft2ToScaleLeft()));
    }
}
