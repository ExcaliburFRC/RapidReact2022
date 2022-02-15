package io.excaliburfrc.robot.commands.auto.shoot2;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.ShootBallsCommand;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Shooter;

/**
 * 0. (6, 4.1, 230)
 *
 * <p>1. go to THEIR_CARGO_5
 *
 * <p>2. go to (4.7, 2.5, 225)
 *
 * <p>3. Drive and collect OUR_CARGO_5
 *
 * <p>4. go to OUR_BOTTOM_FENDER
 *
 * <p>5. shoot 2
 */
public class PushTheirBallAuto extends SequentialCommandGroup {
  private static final Pose2d START = new Pose2d(6, 4.1, degrees(225));
  private static final Pose2d MIDDLE_POINT = new Pose2d(4.7, 2.5, degrees(300));

  public PushTheirBallAuto(Drive drive, Shooter shooter, Intake intake, LEDs leds) {
    super(
        drive.resetOdometryCommand(START),
        drive.followTrajectoryCommand(
            generateTrajectory(START, innerWaypoints(THEIR_CARGO_5), MIDDLE_POINT, FORWARD())),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(
                    MIDDLE_POINT, innerWaypoints(OUR_CARGO_5), OUR_BOTTOM_FENDER, FORWARD()))),
        intake.intakeBallCommand(),
        new ShootBallsCommand(intake, shooter, leds));
  }
}
