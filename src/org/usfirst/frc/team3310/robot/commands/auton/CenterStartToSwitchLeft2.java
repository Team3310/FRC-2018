package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.CenterStartToSwitchLeft;
import org.usfirst.frc.team3310.paths.auton.PyramidToSwitchLeft;
import org.usfirst.frc.team3310.paths.auton.SwitchLeftToCenterStart;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CenterStartToSwitchLeft2 extends CommandGroup {

    public CenterStartToSwitchLeft2() {
    	addSequential(new CenterStartToSwitch2(new CenterStartToSwitchLeft(), new SwitchLeftToCenterStart(), new PyramidToSwitchLeft()));
    }
}
