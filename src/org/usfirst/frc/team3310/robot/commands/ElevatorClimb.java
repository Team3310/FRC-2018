package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.subsystems.Elevator;
import org.usfirst.frc.team3310.robot.subsystems.Ramp;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ElevatorClimb extends CommandGroup {

    public ElevatorClimb() {
        addSequential(new ElevatorSpeedShift(Elevator.SpeedShiftState.LO));
        addSequential(new RampSetLatchPosition(Ramp.RampLatch.DEPLOYED));
        addSequential(new ElevatorSetSpeed(Elevator.CLIMB_SPEED));
    }
}
