package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.CenterStartToSwitchRight;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CenterStartToSwitchRight1 extends CommandGroup {

    public CenterStartToSwitchRight1() {
    	addSequential(new CenterStartToSwitch1(new CenterStartToSwitchRight(), true));
     }
}
