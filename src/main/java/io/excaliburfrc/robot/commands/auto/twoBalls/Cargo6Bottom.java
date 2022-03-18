package io.excaliburfrc.robot.commands.auto.twoBalls;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

public class Cargo6Bottom extends SequentialCommandGroup {
  private static final Pose2d TOP_SHOOT = new Pose2d(7.5, 2.82, degrees(70));
  private static final Pose2d START = new Pose2d(6.7, 2.55, degrees(215));

  public Cargo6Bottom(Drive drive, Superstructure superstructure, LEDs leds) {
    super(
        drive.resetOdometryCommand(START),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(START, innerWaypoints(OUR_CARGO_6), TOP_SHOOT, FORWARD())),
            superstructure.intakeBallCommand()),
        superstructure.shootBallsCommand(leds));
  }
}
