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

public class ballFive extends SequentialCommandGroup {
  static final Pose2d start = new Pose2d(6.952, 2.509, Rotation2d.fromDegrees(200));
  static final Pose2d end = new Pose2d(7.9, 2.6, Rotation2d.fromDegrees(55));
  static final Translation2d ball1 = Trajectories.OUR_CARGO_5;

  public ballFive(Drive drive, LEDs leds, Superstructure superstructure) {
    super(
          drive.resetOdometryCommand(start),
          drive.followTrajectoryCommand(TrajectoryGenerator.generateTrajectory(
                start, innerWaypoints(ball1), end, FORWARD)
          ).alongWith(superstructure.intakeBallCommand())
                .andThen(superstructure.shootBallsCommand(leds))
    );
  }
}
