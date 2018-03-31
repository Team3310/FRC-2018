package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.CenterStartToSwitchRight;
import org.usfirst.frc.team3310.paths.auton.PyramidToSwitchRight;
import org.usfirst.frc.team3310.paths.auton.SwitchRightToCenterStart;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CenterStartToSwitchRight2 extends CommandGroup {

    public CenterStartToSwitchRight2() {
    	addSequential(new CenterStartToSwitch2(new CenterStartToSwitchRight(), new SwitchRightToCenterStart(), new PyramidToSwitchRight(), true));
    }
}
