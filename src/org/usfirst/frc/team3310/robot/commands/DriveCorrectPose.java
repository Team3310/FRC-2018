package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.utility.control.RobotState;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveCorrectPose extends Command
{
    protected RigidTransform2d correction;

    public DriveCorrectPose(RigidTransform2d correction) {
        this.correction = correction;
		requires(Robot.drive);
	}

	@Override
	protected void initialize() {
        RobotState rs = RobotState.getInstance();
        rs.reset(Timer.getFPGATimestamp(), rs.getLatestFieldToVehicle().getValue().transformBy(this.correction));
	}

	@Override
	protected void execute() {
		
	}

	@Override
	protected boolean isFinished() {
		return true;
	}

	@Override
	protected void end() {
		
	}

	@Override
	protected void interrupted() {
			
	}
}