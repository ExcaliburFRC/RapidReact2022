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
 * 0. (7.7, 1.9, 260.0)
 *
 * <p>1. Drive and collect OUR_4
 *
 * <p>2. Drive to OUR_BOTTOM_FENDER
 *
 * <p>3. Shoot 2
 */
public class Cargo4BottomShoot2 extends SequentialCommandGroup {
  private static final Pose2d START = new Pose2d(7.7, 1.9, degrees(260));

  public Cargo4BottomShoot2(Drive drive, Shooter shooter, Intake intake, LEDs leds) {
    super(
        drive.resetOdometryCommand(START),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(
                    START, innerWaypoints(OUR_CARGO_4), OUR_BOTTOM_FENDER, FORWARD())),
            intake.intakeBallCommand()),
        new ShootBallsCommand(intake, shooter, leds));
  }
}
