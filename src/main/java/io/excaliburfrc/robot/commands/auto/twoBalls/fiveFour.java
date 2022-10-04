package io.excaliburfrc.robot.commands.auto.twoBalls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

import io.excaliburfrc.robot.commands.auto.Trajectories;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

public class fiveFour extends SequentialCommandGroup {
static final Pose2d START = new Pose2d(8.2, 2.40, Rotation2d.fromDegrees(90));
  static final Pose2d ball1 = new Pose2d(Trajectories.OUR_CARGO_5.getX(), Trajectories.OUR_CARGO_5.getY(), fromDegrees(200));
  static final Pose2d ball2 = new Pose2d(Trajectories.OUR_CARGO_4.getX(), Trajectories.OUR_CARGO_4.getY(), fromDegrees(0));
  static final Pose2d STOP1 = new Pose2d(8.168, 1.676,START.getRotation());

  public fiveFour(Drive drive, LEDs leds, Superstructure superstructure) {
      super(
            drive.resetOdometryCommand(START),
            superstructure.shootBallsCommand(leds),
            drive.followTrajectoryCommand(START, innerWaypoints(), STOP1, REVERSE),
            new SequentialCommandGroup(
                  // set the end velocity to max vel so the robot doesn't pause
                  new InstantCommand(()-> FORWARD.setEndVelocity(FORWARD.getMaxVelocity())),
                  drive.followTrajectoryCommand(
                        STOP1, innerWaypoints(), ball1, FORWARD),
                  drive.followTrajectoryCommand(
                        ball1, innerWaypoints(), ball2, FORWARD),
                  // set the end velocity back to normal so the robot doesn't hit the Fender
                  new InstantCommand(()-> FORWARD.setEndVelocity(0)),
                  drive.followTrajectoryCommand(
                        ball2, innerWaypoints(), START, FORWARD)
                  ).alongWith(superstructure.intakeBallCommand().andThen(superstructure.intakeBallCommand())),
            superstructure.shootBallsCommand(leds));
    }
}
