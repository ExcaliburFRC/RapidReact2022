package io.excaliburfrc.robot.commands.auto.twoBalls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

public class FiveTerminal extends SequentialCommandGroup {
  static final Pose2d start = new Pose2d(7.1, 2.4, Rotation2d.fromDegrees(24));
  static final Pose2d stop1 = new Pose2d(6, 1, Rotation2d.fromDegrees(90));
  static final Translation2d ball1 = new Translation2d(5.18, 1.9);
  static final Pose2d ball2 = new Pose2d(1.15, 1.115, Rotation2d.fromDegrees(-160));
  static final Pose2d stop2 = new Pose2d(1.15, 2.115, Rotation2d.fromDegrees(-40));

  public FiveTerminal(Drive drive, Superstructure superstracture, LEDs leds){
    super(
          drive.resetOdometryCommand(start),
          superstracture.shootBallsCommand(leds),
          drive.followTrajectoryCommand(start, innerWaypoints(), stop1 ,REVERSE),
          drive.followTrajectoryCommand(stop1, innerWaypoints(ball1), ball2, FORWARD)
                .alongWith(superstracture.intakeBallCommand()
                      .andThen(superstracture.intakeBallCommand())),
          drive.followTrajectoryCommand(ball2, innerWaypoints(), stop2, REVERSE),
          drive.followTrajectoryCommand(stop2, innerWaypoints(), start, FORWARD),
          superstracture.shootBallsCommand(leds)
    );
  }



}
