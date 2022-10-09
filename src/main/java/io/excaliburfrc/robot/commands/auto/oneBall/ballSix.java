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

public class ballSix extends SequentialCommandGroup {
  static final Pose2d start = new Pose2d(7.213, 4.925, Rotation2d.fromDegrees(135));
  static final Pose2d end = new Pose2d(7.83, 4.925, Rotation2d.fromDegrees(350));
  static final Translation2d ball1 = Trajectories.OUR_CARGO_6;
  static final Translation2d stop1 = new Translation2d(6.735, 5.202);

  public ballSix(Drive drive, LEDs leds, Superstructure superstructure) {
    super(
          drive.resetOdometryCommand(start),
          superstructure.shootBallsCommand(leds),
          drive.followTrajectoryCommand(
                      start, innerWaypoints(stop1, ball1), end, FORWARD)
                .alongWith(superstructure.intakeBallCommand())
                .andThen(superstructure.shootBallsCommand(leds))
    );
  }
}
//I first shoot two balls and then I go a little bit back to stop1 so I can take a turn
// and then go to ball 6 and the go back to shoot one more ball
