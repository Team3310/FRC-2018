package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Elevator.ElevatorSpeedShiftState;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ForksIncrementWinchSpeed extends Command {
	
	private double increment;

    public ForksIncrementWinchSpeed(double increment) {
    	this.increment = increment;
        requires(Robot.forks);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.forks.incrementWinchHoldSpeed(increment);
    	Robot.forks.setWinchSpeed(Robot.forks.getWinchHoldSpeed());
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
    	Robot.forks.setWinchSpeed(0);
    }
}
