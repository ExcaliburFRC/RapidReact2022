package io.excaliburfrc.robot.commands.auto.twoBalls;

import static io.excaliburfrc.robot.commands.auto.Trajectories.OUR_CARGO_4;
import static io.excaliburfrc.robot.commands.auto.Trajectories.OUR_CARGO_5;
import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.auto.Trajectories;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

public class fiveFour extends SequentialCommandGroup {
  //static final Pose2d start = new Pose2d(6.952, 2.509, Rotation2d.fromDegrees(200));
  static final Pose2d START = new Pose2d(8.2, 2.40, Rotation2d.fromDegrees(90));
  static final Translation2d ball1 = Trajectories.OUR_CARGO_5;
  static final Translation2d ball2 = Trajectories.OUR_CARGO_4;
  static final Pose2d STOP1 = new Pose2d(8.168, 1.676,START.getRotation());
  static final Translation2d STOP2 = new Translation2d(6.715, 1.057);
  static final Translation2d STOP3 = new Translation2d(7.85, 0.269);

  public fiveFour(Drive drive, LEDs leds, Superstructure superstructure) {
    super(
          drive.resetOdometryCommand(START),
          superstructure.shootBallsCommand(leds),
          drive.followTrajectoryCommand(START,innerWaypoints(),STOP1,REVERSE()),
          drive.followTrajectoryCommand(STOP1,innerWaypoints(),
                new Pose2d(ball1,Rotation2d.fromDegrees(200)),FORWARD()).alongWith(
                      superstructure.intakeBallCommand()),
          drive.followTrajectoryCommand(new Pose2d(ball1,Rotation2d.fromDegrees(200)),
                innerWaypoints(),new Pose2d(ball2,
                      Rotation2d.fromDegrees(0)),FORWARD()).alongWith(
                            superstructure.intakeBallCommand()),
          drive.followTrajectoryCommand(new Pose2d(ball2, Rotation2d.fromDegrees(0)),
                innerWaypoints(), START, FORWARD()),
          superstructure.shootBallsCommand(leds));
  }
}
