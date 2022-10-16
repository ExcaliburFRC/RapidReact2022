package io.excaliburfrc.robot.commands.auto.oneBall;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.auto.Trajectories;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

public class BallFive extends SequentialCommandGroup {
  static final Pose2d START = new Pose2d(7.18, 2.8, Rotation2d.fromDegrees(38.39));
  static final Translation2d BALL_1 = Trajectories.OUR_CARGO_5;
  static final Translation2d STOP_1 = new Translation2d(5.61, 1.16);
  static final Pose2d STOP_2 = new Pose2d(4.03, 0.93, Rotation2d.fromDegrees(29.42));

  public BallFive(Drive drive, LEDs leds, Superstructure superstructure) {
    super(
          drive.resetOdometryCommand(START),
          superstructure.shootBallsCommand(leds),
          drive.followTrajectoryCommand(START, innerWaypoints(STOP_1), STOP_2, REVERSE),
          drive.followTrajectoryCommand(STOP_2, innerWaypoints(BALL_1), START, FORWARD)
                .alongWith(superstructure.intakeBallsCommand()),
          superstructure.shootBallsCommand(leds));
  }
}
