package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ElevatorClimb extends CommandGroup {

    public ElevatorClimb() {
        addSequential(new ElevatorSpeedShift(Elevator.SpeedShiftState.LO));
        addSequential(new ElevatorSetSpeed(Elevator.CLIMB_SPEED));
    }
}
