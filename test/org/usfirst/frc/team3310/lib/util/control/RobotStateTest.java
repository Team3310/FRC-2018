package org.usfirst.frc.team3310.lib.util.control;

import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.usfirst.frc.team3310.utility.control.RobotState;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Rotation2d;
import org.usfirst.frc.team3310.utility.math.Translation2d;

public class RobotStateTest {

    @Test 
    public void testCorrection() {
        RobotState robotState = RobotState.getInstance();
        
        double startX = 100;
        double startY = 200;
        double startAngleDegrees = 90;
        robotState.reset(0, new RigidTransform2d(new Translation2d(startX,startY), Rotation2d.fromDegrees(startAngleDegrees)));
        
        assertTrue(robotState.getLatestFieldToVehicle().getValue().getTranslation().x() == startX);
        assertTrue(robotState.getLatestFieldToVehicle().getValue().getTranslation().y() == startY);
        assertTrue(robotState.getLatestFieldToVehicle().getValue().getRotation().getDegrees() == startAngleDegrees);
        
        double deltaX = 0;
        double deltaY = 20;
        double deltaAngleDegrees = 0;
        
        RigidTransform2d correction = new RigidTransform2d(new Translation2d(deltaX,deltaY), Rotation2d.fromDegrees(deltaAngleDegrees));
        robotState.reset(0, robotState.getLatestFieldToVehicle().getValue().transformBy(correction));
        
        double newX = (startX + Math.cos(Math.toRadians(startAngleDegrees)) * deltaX - Math.sin(Math.toRadians(startAngleDegrees)) * deltaY);
        double newY = (startY + Math.sin(Math.toRadians(startAngleDegrees)) * deltaX + Math.cos(Math.toRadians(startAngleDegrees)) * deltaY);
        
        System.out.println("newXY =" + newX + ", " + newY);
        System.out.println("calcXY =" + robotState.getLatestFieldToVehicle().getValue().getTranslation().x() + ", " + robotState.getLatestFieldToVehicle().getValue().getTranslation().y());
        System.out.println("calcRot =" + robotState.getLatestFieldToVehicle().getValue().getRotation().getDegrees());

        assertTrue(robotState.getLatestFieldToVehicle().getValue().getTranslation().x() == newX);
        assertTrue(robotState.getLatestFieldToVehicle().getValue().getTranslation().y() == newY);
        assertTrue(robotState.getLatestFieldToVehicle().getValue().getRotation().getDegrees() == (startAngleDegrees + deltaAngleDegrees));
    }
}
