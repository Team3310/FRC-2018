package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Flipper.FlipperSide;
import org.usfirst.frc.team3310.robot.subsystems.Flipper.FlipperState;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class FlipperFlip extends Command {

	private FlipperSide flipperSide;
	private FlipperState state;
	
    public FlipperFlip(FlipperSide flipperSide, FlipperState state) {
    	this.flipperSide = flipperSide;
    	this.state = state;
        requires(Robot.flipper);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Flipper flip start, time = " + System.currentTimeMillis());
    	Robot.flipper.setPosition(flipperSide, state);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
