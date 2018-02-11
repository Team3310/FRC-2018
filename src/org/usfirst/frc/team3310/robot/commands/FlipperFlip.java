package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Flipper.FlipperSide;
import org.usfirst.frc.team3310.robot.subsystems.Flipper.FlipperState;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class FlipperFlip extends Command {

	private static double FLIP_TIMEOUT = 2.0;
	private FlipperSide flipperSide;
	
    public FlipperFlip(FlipperSide flipperSide) {
    	this.flipperSide = flipperSide;
        requires(Robot.flipper);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.flipper.setPosition(flipperSide, FlipperState.DEPLOYED);
    	setTimeout(FLIP_TIMEOUT);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.flipper.setPosition(flipperSide, FlipperState.RETRACTED);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
