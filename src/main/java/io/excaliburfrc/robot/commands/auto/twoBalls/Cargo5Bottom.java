package io.excaliburfrc.robot.commands.auto.twoBalls;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.*;

public class Cargo5Bottom extends SequentialCommandGroup {

  private static final Pose2d BOTTOM_SHOOT = new Pose2d(7.644466, 3.025257, degrees(70.000000));
  private static final Pose2d START = new Pose2d(6.490951, 2.582143, degrees(227.939995));

  public Cargo5Bottom(Drive drive, Superstructure superstructure, LEDs leds) {
    super(
        drive.resetOdometryCommand(START),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(
                    START, innerWaypoints(OUR_CARGO_5), BOTTOM_SHOOT, FORWARD())),
            superstructure.intakeBallCommand()),
        superstructure.shootBallsCommand(leds));
  }
}
