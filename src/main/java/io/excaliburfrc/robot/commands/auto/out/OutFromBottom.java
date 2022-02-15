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
 * 0. (7.2, 2.1, 230)
 *
 * <p>1. Go to (5, 0.7, 180)
 */
public class OutFromBottom extends SequentialCommandGroup {
  private static final Pose2d START = new Pose2d(7.2, 2.1, degrees(230));
  private static final Pose2d END = new Pose2d(5, 0.7, degrees(180));

  public OutFromBottom(Drive drive, Intake intake, Shooter shooter, LEDs leds) {
    super(
        drive.resetOdometryCommand(START),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(START, innerWaypoints(), END, FORWARD())),
            intake.intakeBallCommand()),
        new ShootBallsCommand(intake, shooter, leds));
  }
}
