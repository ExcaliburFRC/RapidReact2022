package io.excaliburfrc.robot.commands.auto.checkTrajectory;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.Drive;

public class Try1 extends SequentialCommandGroup {

  public Try1(Drive drive) {
    super(
        drive.resetOdometryCommand(new Pose2d(0, 0, degrees(0))),
        drive.followTrajectoryCommand(
            generateTrajectory(
                new Pose2d(0, 0, degrees(0)),
                innerWaypoints(),
                new Pose2d(2, 2, degrees(45)),
                FORWARD())),
        drive.followTrajectoryCommand(
            generateTrajectory(
                new Pose2d(2, 2, degrees(45)),
                innerWaypoints(),
                new Pose2d(1, 0.5, degrees(-45)),
                FORWARD())),
        drive.followTrajectoryCommand(
            generateTrajectory(
                new Pose2d(1, 4, degrees(-45)),
                innerWaypoints(),
                new Pose2d(2, 7, degrees(0)),
                FORWARD())),
        drive.followTrajectoryCommand(
            generateTrajectory(
                new Pose2d(2, 7, degrees(0)),
                innerWaypoints(),
                new Pose2d(6, 1.5, degrees(90)),
                FORWARD())));
  }
}
