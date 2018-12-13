package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.CenterStartToScaleLeft;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeft;
import org.usfirst.frc.team3310.paths.auton.SwitchLeftToScaleRight;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CenterStartToSwitchLeft1ScaleRight1 extends CommandGroup {

    public CenterStartToSwitchLeft1ScaleRight1() {

    	addSequential(new CenterStartToSwitch1Scale1(new CenterStartToScaleLeft(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleRight(), true));

    }
}
