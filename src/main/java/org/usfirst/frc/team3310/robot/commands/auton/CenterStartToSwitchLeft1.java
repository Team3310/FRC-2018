package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.CenterStartToSwitchLeft;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CenterStartToSwitchLeft1 extends CommandGroup {

    public CenterStartToSwitchLeft1() {
    	addSequential(new CenterStartToSwitch1(new CenterStartToSwitchLeft(), true, 1.0));
    }
}
