package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.LeftStartToScaleLeftV2;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeft;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight;
import org.usfirst.frc.team3310.paths.auton.SwitchLeftToScaleRightV2;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftStartToScaleLeft1SwitchRight1V2 extends CommandGroup {

    public LeftStartToScaleLeft1SwitchRight1V2() {
    
    	addSequential(new StartToScale1Switch1(new LeftStartToScaleLeftV2(), new ScaleLeftToSwitchLeft(), new SwitchLeftToScaleRightV2(), new ScaleRightToSwitchRight()));

    }
}
