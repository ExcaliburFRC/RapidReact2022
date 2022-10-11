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
  static final Pose2d START = new Pose2d(7.18, 2.8, Rotation2d.fromDegrees(38.39));
  static final Pose2d STOP_1 = new Pose2d(6.29, 1.03, Rotation2d.fromDegrees(90));
  static final Translation2d BALL_1 = OUR_CARGO_5;
  static final Pose2d BALL_2 = new Pose2d(OUR_TERMINAL_CARGO, Rotation2d.fromDegrees(180));
  static final Pose2d STOP_2 = new Pose2d(2.32, 2.23, Rotation2d.fromDegrees(-90));

  public FiveTerminal(Drive drive, LEDs leds, Superstructure superstructure){
    super(
          drive.resetOdometryCommand(START),
          superstructure.shootBallsCommand(leds),
          drive.followTrajectoryCommand(START, innerWaypoints(), STOP_1, REVERSE),
          drive.followTrajectoryCommand(STOP_1, innerWaypoints(BALL_1), BALL_2, FORWARD)
                .alongWith(superstructure.intakeBallCommand().andThen(superstructure.intakeBallCommand())),
          drive.followTrajectoryCommand(BALL_2, innerWaypoints(), STOP_2, REVERSE),
          drive.followTrajectoryCommand(STOP_2, innerWaypoints(), START, FORWARD),
          superstructure.shootBallsCommand(leds)
    );
  }
}
