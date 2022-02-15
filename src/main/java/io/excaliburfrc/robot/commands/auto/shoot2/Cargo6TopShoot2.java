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
 * 0. (6.3, 5.3, 155)
 *
 * <p>1. Drive and collect cargo 6
 *
 * <p>2. Drive to our top fender
 *
 * <p>3. Shoot 2
 */
public class Cargo6TopShoot2 extends SequentialCommandGroup {
  private static final Pose2d START = new Pose2d(6.3, 5.3, degrees(155));

  public Cargo6TopShoot2(Drive drive, Intake intake, Shooter shooter, LEDs leds) {
    super(
        drive.resetOdometryCommand(START),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(
                    START,
                    innerWaypoints(OUR_CARGO_6),
                    OUR_TOP_FENDER,
                    FORWARD())), // :TODO: if we don't ger the ball we can run to the fender with
            // the intake open (relevant for most of the autos commands)
            intake.intakeBallCommand()),
        new ShootBallsCommand(intake, shooter, leds));
  }
}
