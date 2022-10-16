package io.excaliburfrc.robot.commands.auto.oneBall;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

public class BallFour extends SequentialCommandGroup {
  static final Pose2d start = new Pose2d(7.95, 2.6, Rotation2d.fromDegrees(90));
  static final Pose2d stop1 = new Pose2d(6.6, 1.65, Rotation2d.fromDegrees(0));
  static final Pose2d ball1 = new Pose2d(OUR_CARGO_4.getX(), OUR_CARGO_4.getY() + 0.2, Rotation2d.fromDegrees(-90));
  static final TrajectoryConfig Fconfig = new TrajectoryConfig(1.35, 1.5).setReversed(false);
  static final TrajectoryConfig Rconfig = new TrajectoryConfig(1.35, 1.5).setReversed(true);

  public BallFour(Drive drive, LEDs leds, Superstructure superstructure) {
    super(
          superstructure.shootBallsCommand(leds),
          drive.resetOdometryCommand(start),
          drive.followTrajectoryCommand(start, innerWaypoints(), stop1, Rconfig),
          drive.followTrajectoryCommand(stop1, innerWaypoints(), ball1, Fconfig)
                .alongWith(superstructure.intakeBallsCommand()),
          drive.followTrajectoryCommand(ball1, innerWaypoints(), stop1, Rconfig),
          drive.followTrajectoryCommand(stop1, innerWaypoints(), start, Fconfig),
          superstructure.shootBallsCommand(leds)
    );
  }
}
