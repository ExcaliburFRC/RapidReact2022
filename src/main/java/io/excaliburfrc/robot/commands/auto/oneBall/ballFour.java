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

import static io.excaliburfrc.robot.commands.auto.Trajectories.FORWARD;
import static io.excaliburfrc.robot.commands.auto.Trajectories.innerWaypoints;

public class ballFour extends SequentialCommandGroup {
  static final Pose2d start = new Pose2d(8.123, 2.551, Rotation2d.fromDegrees(80));
  static final Pose2d end = new Pose2d(7.83, 2.486, Rotation2d.fromDegrees(50));
  static final Translation2d ball1 = Trajectories.OUR_CARGO_4;
  static final Translation2d stop1 = new Translation2d(7.413, 2.057);

  public ballFour(Drive drive, LEDs leds, Superstructure superstructure) {
    super(
          drive.resetOdometryCommand(start),
          drive.followTrajectoryCommand(TrajectoryGenerator.generateTrajectory(
                      start, innerWaypoints(ball1, stop1), end, FORWARD)
                ).alongWith(superstructure.intakeBallCommand())
                .andThen(superstructure.shootBallsCommand(leds))
    );
  }
}
