package org.usfirst.frc.team3310.robot.commands.auton;

import org.usfirst.frc.team3310.paths.auton.RightStartToScaleRightV2;
import org.usfirst.frc.team3310.paths.auton.ScaleLeftToSwitchLeftV2;
import org.usfirst.frc.team3310.paths.auton.ScaleRightToSwitchRight3Cube;
import org.usfirst.frc.team3310.paths.auton.SwitchRightToScaleLeftV2;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightStartToScaleRight1SwitchLeft1V2 extends CommandGroup {

    public RightStartToScaleRight1SwitchLeft1V2() {
    
    	addSequential(new StartToScale1Switch1(new RightStartToScaleRightV2(), new ScaleRightToSwitchRight3Cube(), new SwitchRightToScaleLeftV2(), new ScaleLeftToSwitchLeftV2()));

    }
}
