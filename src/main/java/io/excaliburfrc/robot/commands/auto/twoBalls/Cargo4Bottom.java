package io.excaliburfrc.robot.commands.auto.twoBalls;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.*;

public class Cargo4Bottom extends SequentialCommandGroup {
  private static final Pose2d START = new Pose2d(7.579407, 1.767444, degrees(272.000000));
  private static final Pose2d BOTTOM_SHOOT = new Pose2d(7.644466, 3.025257, degrees(70.000000));

  public Cargo4Bottom(Drive drive, Superstructure superstructure, LEDs leds) {
    super(
        drive.resetOdometryCommand(START),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(START, innerWaypoints(OUR_CARGO_4), BOTTOM_SHOOT, FORWARD())),
            superstructure.intakeBallCommand()),
        superstructure.shootBallsCommand(leds));
  }
}
