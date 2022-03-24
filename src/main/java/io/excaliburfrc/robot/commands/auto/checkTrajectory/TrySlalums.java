package io.excaliburfrc.robot.commands.auto.checkTrajectory;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.Drive;

public class TrySlalums extends SequentialCommandGroup {
  public TrySlalums(Drive drive) {
    super(
        drive.resetOdometryCommand(new Pose2d(0, 0, degrees(90))),
        drive.followTrajectoryCommand(
            generateTrajectory(
                new Pose2d(0, 0, degrees(90)),
                innerWaypoints(),
                new Pose2d(1, 3.5, degrees(90)),
                FORWARD())),
        drive.followTrajectoryCommand(
            generateTrajectory(
                new Pose2d(1, 3.5, degrees(90)),
                innerWaypoints(),
                new Pose2d(2, 0.5, degrees(90)),
                REVERSE())),
        drive.followTrajectoryCommand(
            generateTrajectory(
                new Pose2d(2, 0.5, degrees(90)),
                innerWaypoints(new Translation2d(3, 3.5)),
                new Pose2d(4, 0.5, degrees(0)),
                FORWARD())),
        drive.followTrajectoryCommand(
            generateTrajectory(
                new Pose2d(4, 0.5, degrees(0)),
                innerWaypoints(new Translation2d(5, 3)),
                new Pose2d(6, 1.5, degrees(45)),
                REVERSE())));
  }
}
