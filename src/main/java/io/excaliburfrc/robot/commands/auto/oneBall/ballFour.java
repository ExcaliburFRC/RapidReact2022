package io.excaliburfrc.robot.commands.auto.oneBall;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.auto.Trajectories;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

public class ballFour extends SequentialCommandGroup {
  static final Pose2d start = new Pose2d(7.95, 2.6, Rotation2d.fromDegrees(90));
  static final Pose2d stop1 = new Pose2d(6.6, 1.65, Rotation2d.fromDegrees(0));
  static final Pose2d ball1 = new Pose2d(OUR_CARGO_4, Rotation2d.fromDegrees(-90));

  public ballFour(Drive drive, LEDs leds, Superstructure superstructure) {
    super(
          superstructure.shootBallsCommand(leds),
          drive.resetOdometryCommand(start),
          drive.followTrajectoryCommand(start, innerWaypoints(), stop1, REVERSE),
          drive.followTrajectoryCommand(stop1, innerWaypoints(), ball1, FORWARD)
                .alongWith(superstructure.intakeBallCommand()),
          drive.followTrajectoryCommand(ball1, innerWaypoints(), stop1, REVERSE),
          drive.followTrajectoryCommand(stop1, innerWaypoints(), start, FORWARD),
          superstructure.shootBallsCommand(leds)
    );
  }
}
