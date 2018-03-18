package org.usfirst.frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class IntakeCubeAndLiftAbortDriveAsymmetric extends CommandGroup {

    public IntakeCubeAndLiftAbortDriveAsymmetric(boolean extraIntake) {
        addSequential(new IntakeSetSpeedFrontSensorOffAsymmetric(1.0, 0.6));
        addSequential(new DriveForcePathDone());
//        addSequential(new ElevatorSetPositionMP(Elevator.AFTER_INTAKE_POSITION_INCHES));
//        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SLOW_SPEED, 0.2));

//        addSequential(new IntakeSetSpeedFrontSensorOff(Intake.INTAKE_LOAD_SPEED));
//        addSequential(new DriveForcePathDone());
//        if (extraIntake) {
//            addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SPEED, 0.4));
//        }
    }
    
    protected void initialize() {
    	System.out.println("IntakeCubeAndLiftAbortDrive initialized");
    }
    
    protected void interrupted() {
    	System.out.println("IntakeCubeAndLiftAbortDrive interrupted");
    }
}
