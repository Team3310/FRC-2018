package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Drive;
import org.usfirst.frc.team3310.utility.control.RobotState;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveResetPoseFromPath extends Command
{
    protected PathContainer pathContainer;

    public DriveResetPoseFromPath(PathContainer pathContainer) {
        this.pathContainer = pathContainer;
		requires(Robot.drive);
	}

	@Override
	protected void initialize() {
        RigidTransform2d startPose = pathContainer.getStartPose();
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
        Robot.drive.resetGyro(startPose.getRotation().getDegrees());
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