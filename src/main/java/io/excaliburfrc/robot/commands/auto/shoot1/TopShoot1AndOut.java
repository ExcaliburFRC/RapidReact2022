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
 * 0. OUR_TOP_FENDER
 *
 * <p>1. Shoot 1
 *
 * <p>2. Drive away to ~(3.8, 4.4, 180)
 */
public class TopShoot1AndOut extends SequentialCommandGroup {
  private static final Pose2d END_OUTSIDE_TARMAC = new Pose2d(4.1, 4.5, degrees(0.0));

  public TopShoot1AndOut(Drive drive, Shooter shooter, Intake intake, LEDs leds) {
    super(
        drive.resetOdometryCommand(OUR_TOP_FENDER),
        new ShootBallsCommand(intake, shooter, leds),
        drive.followTrajectoryCommand(
            generateTrajectory(OUR_TOP_FENDER, innerWaypoints(), END_OUTSIDE_TARMAC, FORWARD())));
  }
}
