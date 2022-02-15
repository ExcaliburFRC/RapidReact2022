package io.excaliburfrc.robot.commands.auto.shoot1;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.ShootBallsCommand;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Shooter;

/**
 * 0. OUR_BOTTOM_FENDER
 *
 * <p>1. Shoot 1
 *
 * <p>2. Drive away to ~(5.5, 0.7, 180)
 */
public class BottomShoot1AndOut extends SequentialCommandGroup {
  private static final Pose2d END_OUTSIDE_TARMAC = new Pose2d(5.5, 0.7, degrees(180));

  public BottomShoot1AndOut(Drive drive, Shooter shooter, Intake intake, LEDs leds) {
    super(
        drive.resetOdometryCommand(OUR_BOTTOM_FENDER),
        new ShootBallsCommand(intake, shooter, leds),
        drive.followTrajectoryCommand(
            generateTrajectory(
                OUR_BOTTOM_FENDER, innerWaypoints(), END_OUTSIDE_TARMAC, FORWARD())));
  }
}
