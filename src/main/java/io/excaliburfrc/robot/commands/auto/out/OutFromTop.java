package io.excaliburfrc.robot.commands.auto.out;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.ShootBallsCommand;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Shooter;

/**
 * 0. (6, 4.5, 180)
 *
 * <p>1. Go to (3.8, 4.4, 180)
 */
public class OutFromTop extends SequentialCommandGroup {
  private static final Pose2d START = new Pose2d(6, 4.5, degrees(180));
  private static final Pose2d END = new Pose2d(3.8, 4.4, degrees(180));

  public OutFromTop(Drive drive, Intake intake, Shooter shooter, LEDs leds) {
    super(
        drive.resetOdometryCommand(START),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(START, innerWaypoints(), END, REVERSE())),
            intake.intakeBallCommand()),
        new ShootBallsCommand(intake, shooter, leds));
  }
}
