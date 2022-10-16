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

public class BallSix extends SequentialCommandGroup {
  static final Pose2d start = OUR_TOP_FENDER;
  static final Translation2d ball1 = Trajectories.OUR_CARGO_6;
  static final Pose2d STOP1 = new Pose2d( 4.86, 7.7, Rotation2d.fromDegrees( -77.12));

  public BallSix(Drive drive, LEDs leds, Superstructure superstructure) {
    super(
          drive.resetOdometryCommand(start),
          superstructure.shootBallsCommand(leds),
          drive.followTrajectoryCommand(
                start, innerWaypoints(), STOP1, REVERSE),
          drive.followTrajectoryCommand(STOP1, innerWaypoints(ball1), start, FORWARD)
                .alongWith(superstructure.intakeBallsCommand()),
    superstructure.shootBallsCommand(leds));
  }
}
//I first shoot two balls and then I go a little bit back to stop1 so I can take a turn
// and then go to ball 6 and the go back to shoot one more ball
