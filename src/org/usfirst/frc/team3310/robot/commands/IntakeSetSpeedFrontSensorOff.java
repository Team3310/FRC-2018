package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;

/**
 *
 */
public class IntakeSetSpeedFrontSensorOff extends ExtraTimeoutCommand {
	
	private double speed;
	private boolean cubeDetected;
	private static final double EXTRA_INTAKE_TIME = 0.1;
	private static final double TIMEOUT = 10.0;

    public IntakeSetSpeedFrontSensorOff(double speed) {
    	this.speed = speed;
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	resetExtraTimer();
    	setTimeout(TIMEOUT);
		cubeDetected = false;
    	Robot.intake.setSpeed(speed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (cubeDetected == false && Robot.elevator.getFrontIntakeSensor()) {
    		startExtraTimeout(EXTRA_INTAKE_TIME);
    		cubeDetected = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isExtraTimedOut() || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intake.setSpeed(0);
 }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}