package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorSetPositionPID extends Command {
	
	private double targetPositionInches;

    public ElevatorSetPositionPID(double targetPositionInches) {
    	this.targetPositionInches = targetPositionInches;
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.setPositionPID(targetPositionInches);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(Robot.elevator.getPositionInches() - this.targetPositionInches) < 1.0;
    }

    // Called once after isFinished returns true
    protected void end() {
		Robot.elevator.setPositionPID(Robot.elevator.getPositionInches());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
