package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class IntakeCubeAndLiftAbortDrive extends CommandGroup {

    public IntakeCubeAndLiftAbortDrive(boolean extraIntake) {
        addSequential(new IntakeSetSpeedFrontSensorOff(Intake.INTAKE_LOAD_SPEED));
        addSequential(new DriveForcePathDone());
        if (extraIntake) {
            addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SPEED, 0.4));
        }
    }
    
    protected void initialize() {
    	System.out.println("IntakeCubeAndLiftAbortDrive initialized");
    }
    
    protected void interrupted() {
    	System.out.println("IntakeCubeAndLiftAbortDrive interrupted");
    }
}
