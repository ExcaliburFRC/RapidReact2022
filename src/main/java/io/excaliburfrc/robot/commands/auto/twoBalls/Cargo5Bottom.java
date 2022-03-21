package io.excaliburfrc.robot.commands.auto.twoBalls;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.*;

public class Cargo5Bottom extends SequentialCommandGroup {

  private static final Pose2d BOTTOM_SHOOT = new Pose2d(7.5, 2.82, degrees(70));
  private static final Pose2d START = new Pose2d(6.7, 2.55, degrees(215));

  public Cargo5Bottom(Drive drive, Superstructure superstructure, LEDs leds) {
    super(
        drive.resetOdometryCommand(START),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(
                    START, innerWaypoints(new Translation2d(4.5, 1.5)), BOTTOM_SHOOT, FORWARD())),
            superstructure.intakeBallCommand()),
        superstructure.shootBallsCommand(leds));
  }
}
