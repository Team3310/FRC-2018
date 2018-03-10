package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class IntakeCubeAndLift extends CommandGroup {

    public IntakeCubeAndLift() {
        addSequential(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
//        addSequential(new ElevatorAutoZero(true));
        addSequential(new IntakeSetSpeedFrontSensorOff(Intake.INTAKE_LOAD_SPEED));
        addSequential(new ElevatorSetPositionMP(Elevator.AFTER_INTAKE_POSITION_INCHES));
        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SLOW_SPEED, 0.2));
    }
}
