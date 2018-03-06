package org.usfirst.frc.team3310.lib.util.control;

import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.paths.test.LeftTurn;
import org.usfirst.frc.team3310.paths.test.LeftTurnRadius;
import org.usfirst.frc.team3310.paths.test.LeftTurnRadiusReversed;
import org.usfirst.frc.team3310.paths.test.LeftTurnRadiusStart90;
import org.usfirst.frc.team3310.paths.test.LeftTurnReversed;
import org.usfirst.frc.team3310.paths.test.SCurveReversed;
import org.usfirst.frc.team3310.robot.Constants;
import org.usfirst.frc.team3310.utility.ReflectingCSVWriter;
import org.usfirst.frc.team3310.utility.control.Lookahead;
import org.usfirst.frc.team3310.utility.control.PathFollower;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Twist2d;

public class PathFollowerTest {

    static final PathFollower.Parameters kParameters = new PathFollower.Parameters(
            new Lookahead(16.0, 16.0, 0.0, 120.0),
            0.0, // Inertia gain
            0.75, // Profile kp
            0.03, // Profile ki
            0.02, // Profile kv
            1.0, // Profile kffv
            0.0, // Profile kffa
            Constants.kPathFollowingMaxVel, // Profile max abs vel
            Constants.kPathFollowingMaxAccel, // Profile max abs accel
            Constants.kPathFollowingGoalPosTolerance,
            Constants.kPathFollowingGoalVelTolerance,
            Constants.kPathStopSteeringDistance
    );

    @Test 
    public void testLeftTurn() {
        PathContainer container = new LeftTurn();
        PathFollower controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);

        ReflectingCSVWriter<PathFollower.DebugOutput> writer = new ReflectingCSVWriter<PathFollower.DebugOutput>(
                "testLeftTurn.csv", PathFollower.DebugOutput.class);

        final double dt = 0.01;

        RigidTransform2d robot_pose = container.getStartPose();
        double t = 0;
        double displacement = 0.0;
        double velocity = 0.0;
        while (!controller.isFinished() && t < 10.0) {
            // Follow the path
            Twist2d command = controller.update(t, robot_pose, displacement, velocity);
            writer.add(controller.getDebug());
            robot_pose = robot_pose.transformBy(RigidTransform2d.exp(command.scaled(dt)));

            t += dt;
            final double prev_vel = velocity;
            velocity = command.dx;
            displacement += velocity * dt;

            System.out.println("t = " + t + ", displacement " + displacement + ", lin vel " + command.dx + ", lin acc "
                    + (velocity - prev_vel) / dt + ", ang vel " + command.dtheta + ", pose " + robot_pose + ", CTE "
                    + controller.getCrossTrackError() + ", ATE " + controller.getAlongTrackError());
        }
        writer.flush();
        System.out.println(robot_pose);
        assertTrue(controller.isFinished());
        assertTrue(controller.getAlongTrackError() < 8.0);
     }

    @Test
    public void testLeftTurnRadius() {
        PathContainer container = new LeftTurnRadius();
        PathFollower controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);
 
        ReflectingCSVWriter<PathFollower.DebugOutput> writer = new ReflectingCSVWriter<PathFollower.DebugOutput>(
                "testLeftTurnRadius.csv", PathFollower.DebugOutput.class);

        final double dt = 0.01;

        RigidTransform2d robot_pose = container.getStartPose();
        double t = 0;
        double displacement = 0.0;
        double velocity = 0.0;
        while (!controller.isFinished() && t < 25.0) {
            // Follow the path
            Twist2d command = controller.update(t, robot_pose, displacement, velocity);
            writer.add(controller.getDebug());
            robot_pose = robot_pose.transformBy(RigidTransform2d.exp(command.scaled(dt)));

            t += dt;
            final double prev_vel = velocity;
            velocity = command.dx;
            displacement += velocity * dt;

            System.out.println("t = " + t + ", displacement " + displacement + ", lin vel " + command.dx + ", lin acc "
                    + (velocity - prev_vel) / dt + ", ang vel " + command.dtheta + ", pose " + robot_pose + ", CTE "
                    + controller.getCrossTrackError() + ", ATE " + controller.getAlongTrackError());
        }
        writer.flush();
        System.out.println(robot_pose);
        assertTrue(controller.isFinished());
        assertTrue(controller.getAlongTrackError() < 1.0);
        assertTrue(controller.getCrossTrackError() < 1.0);
    }

    @Test
    public void testLeftTurnRadiusStart90() {
        PathContainer container = new LeftTurnRadiusStart90();
        PathFollower controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);
 
        ReflectingCSVWriter<PathFollower.DebugOutput> writer = new ReflectingCSVWriter<PathFollower.DebugOutput>(
                "testLeftTurnRadiusStart90.csv", PathFollower.DebugOutput.class);

        final double dt = 0.01;

        RigidTransform2d robot_pose = container.getStartPose();
        double t = 0;
        double displacement = 0.0;
        double velocity = 0.0;
        while (!controller.isFinished() && t < 25.0) {
            // Follow the path
            Twist2d command = controller.update(t, robot_pose, displacement, velocity);
            writer.add(controller.getDebug());
            robot_pose = robot_pose.transformBy(RigidTransform2d.exp(command.scaled(dt)));

            t += dt;
            final double prev_vel = velocity;
            velocity = command.dx;
            displacement += velocity * dt;

            System.out.println("t = " + t + ", displacement " + displacement + ", lin vel " + command.dx + ", lin acc "
                    + (velocity - prev_vel) / dt + ", ang vel " + command.dtheta + ", pose " + robot_pose + ", CTE "
                    + controller.getCrossTrackError() + ", ATE " + controller.getAlongTrackError());
        }
        writer.flush();
        System.out.println(robot_pose);
        assertTrue(controller.isFinished());
        assertTrue(controller.getAlongTrackError() < 1.0);
        assertTrue(controller.getCrossTrackError() < 1.0);
    }

    @Test
    public void testLeftTurnReversed() {
        PathContainer container = new LeftTurnReversed();
        PathFollower controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);

        ReflectingCSVWriter<PathFollower.DebugOutput> writer = new ReflectingCSVWriter<PathFollower.DebugOutput>(
                "testLeftTurnReversed.csv", PathFollower.DebugOutput.class);

        final double dt = 0.01;

        RigidTransform2d robot_pose = container.getStartPose();
        double t = 0;
        double displacement = 0.0;
        double velocity = 0.0;
        while (!controller.isFinished() && t < 10.0) {
            // Follow the path
            Twist2d command = controller.update(t, robot_pose, displacement, velocity);
            writer.add(controller.getDebug());
            robot_pose = robot_pose.transformBy(RigidTransform2d.exp(command.scaled(dt)));

            t += dt;
            final double prev_vel = velocity;
            velocity = command.dx;
            displacement += velocity * dt;

            System.out.println("t = " + t + ", displacement " + displacement + ", lin vel " + command.dx + ", lin acc "
                    + (velocity - prev_vel) / dt + ", ang vel " + command.dtheta + ", pose " + robot_pose + ", CTE "
                    + controller.getCrossTrackError() + ", ATE " + controller.getAlongTrackError());
        }
        writer.flush();
        System.out.println(robot_pose);
        assertTrue(controller.isFinished());
        assertTrue(controller.getAlongTrackError() < 1.0);
        assertTrue(controller.getCrossTrackError() < 8.0);
    }

    @Test
    public void testLeftTurnRadiusReversed() {
        PathContainer container = new LeftTurnRadiusReversed();
        PathFollower controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);

        ReflectingCSVWriter<PathFollower.DebugOutput> writer = new ReflectingCSVWriter<PathFollower.DebugOutput>(
                "testLeftTurnRadiusReversed.csv", PathFollower.DebugOutput.class);

        final double dt = 0.01;

        RigidTransform2d robot_pose = container.getStartPose();
        double t = 0;
        double displacement = 0.0;
        double velocity = 0.0;
        while (!controller.isFinished() && t < 25.0) {
            // Follow the path
            Twist2d command = controller.update(t, robot_pose, displacement, velocity);
            writer.add(controller.getDebug());
            robot_pose = robot_pose.transformBy(RigidTransform2d.exp(command.scaled(dt)));

            t += dt;
            final double prev_vel = velocity;
            velocity = command.dx;
            displacement += velocity * dt;

            System.out.println("t = " + t + ", displacement " + displacement + ", lin vel " + command.dx + ", lin acc "
                    + (velocity - prev_vel) / dt + ", ang vel " + command.dtheta + ", pose " + robot_pose + ", CTE "
                    + controller.getCrossTrackError() + ", ATE " + controller.getAlongTrackError());
        }
        writer.flush();
        System.out.println(robot_pose);
        assertTrue(controller.isFinished());
        assertTrue(controller.getAlongTrackError() < 1.0);
        assertTrue(controller.getCrossTrackError() < 1.0);
    }

    @Test
    public void testSCurveReversed() {
        PathContainer container = new SCurveReversed();
        PathFollower controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);

        ReflectingCSVWriter<PathFollower.DebugOutput> writer = new ReflectingCSVWriter<PathFollower.DebugOutput>(
                "testSCurveReversed.csv", PathFollower.DebugOutput.class);

        final double dt = 0.01;

        RigidTransform2d robot_pose = container.getStartPose();
        double t = 0;
        double displacement = 0.0;
        double velocity = 0.0;
        while (!controller.isFinished() && t < 25.0) {
            // Follow the path
            Twist2d command = controller.update(t, robot_pose, displacement, velocity);
            writer.add(controller.getDebug());
            robot_pose = robot_pose.transformBy(RigidTransform2d.exp(command.scaled(dt)));

            t += dt;
            final double prev_vel = velocity;
            velocity = command.dx;
            displacement += velocity * dt;

            System.out.println("t = " + t + ", displacement " + displacement + ", lin vel " + command.dx + ", lin acc "
                    + (velocity - prev_vel) / dt + ", ang vel " + command.dtheta + ", pose " + robot_pose + ", CTE "
                    + controller.getCrossTrackError() + ", ATE " + controller.getAlongTrackError());
        }
        writer.flush();
        System.out.println(robot_pose);
        assertTrue(controller.isFinished());
        assertTrue(controller.getAlongTrackError() < 1.0);
        assertTrue(controller.getCrossTrackError() < 1.0);
    }

    @Test
    public void testTwoPaths() {
        PathContainer container = new LeftTurnRadiusReversed();
        PathFollower controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);

        ReflectingCSVWriter<PathFollower.DebugOutput> writer = new ReflectingCSVWriter<PathFollower.DebugOutput>(
                "testTwoPaths.csv", PathFollower.DebugOutput.class);

        final double dt = 0.01;

//        RigidTransform2d robot_pose = container.getStartPose()
//                .transformBy(new RigidTransform2d(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(15.0)));
        RigidTransform2d robot_pose = container.getStartPose();
        double t = 0;
        double displacement = 0.0;
        double velocity = 0.0;
        while (!controller.isFinished() && t < 25.0) {
            // Follow the path
            Twist2d command = controller.update(t, robot_pose, displacement, velocity);
            writer.add(controller.getDebug());
            robot_pose = robot_pose.transformBy(RigidTransform2d.exp(command.scaled(dt)));

            t += dt;
            final double prev_vel = velocity;
            velocity = command.dx;
            displacement += velocity * dt;

            System.out.println("t = " + t + ", displacement " + displacement + ", lin vel " + command.dx + ", lin acc "
                    + (velocity - prev_vel) / dt + ", ang vel " + command.dtheta + ", pose " + robot_pose + ", CTE "
                    + controller.getCrossTrackError() + ", ATE " + controller.getAlongTrackError());
        }
        System.out.println(robot_pose);
        assertTrue(controller.isFinished());
        assertTrue(controller.getAlongTrackError() < 1.0);
        assertTrue(controller.getCrossTrackError() < 1.0);

        displacement = 0.0;
        container = new LeftTurnRadiusStart90();
        controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);
        boolean has_tweaked = false;
        while (!controller.isFinished() && t < 25.0) {
            if (t > 4.2 && !has_tweaked) {
                has_tweaked = true;
                displacement -= 2.0;
                System.out.println("Tweak!");
            }
            
            if (t > 5.6) {
                System.out.println("What");
            }
            
            // Follow the path
            Twist2d command = controller.update(t, robot_pose, displacement, velocity);
            writer.add(controller.getDebug());
            robot_pose = robot_pose.transformBy(RigidTransform2d.exp(command.scaled(dt)));

            t += dt;
            final double prev_vel = velocity;
            velocity = command.dx;
            displacement += velocity * dt;
            
            System.out.println("t = " + t + ", displacement " + displacement + ", lin vel " + command.dx + ", lin acc "
                    + (velocity - prev_vel) / dt + ", ang vel " + command.dtheta + ", pose " + robot_pose + ", CTE "
                    + controller.getCrossTrackError() + ", ATE " + controller.getAlongTrackError());
        }
        writer.flush();
        System.out.println(robot_pose);
        assertTrue(controller.isFinished());
        assertTrue(controller.getAlongTrackError() < 1.0);
        assertTrue(controller.getCrossTrackError() < 1.0);
    }
}