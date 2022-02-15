package io.excaliburfrc.robot.commands.auto.shoot4;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.ShootBallsCommand;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Shooter;

/**
 * 0. (7.7, 1.9, 260)
 *
 * <p>2. go to OUR_CARGO_4
 *
 * <p>3. go to OUR_BOTTOM_FENDER
 *
 * <p>4. shoot x2
 *
 * <p>5. go to OUR_CARGO_5
 *
 * <p>6. go to OUR_TERMINAL_CARGO
 *
 * <p>7. go to OUR_BOTTOM_FENDER
 *
 * <p>8. shoot x2
 */
public class FourBallsAuto extends SequentialCommandGroup {
  private static final Pose2d START = new Pose2d(7.7, 1.9, degrees(260));

  public FourBallsAuto(Drive drive, Shooter shooter, Intake intake, LEDs leds) {
    super(
        drive.resetOdometryCommand(START),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(
                    START, innerWaypoints(OUR_CARGO_4), OUR_BOTTOM_FENDER, FORWARD())),
            intake.intakeBallCommand()),
        new ShootBallsCommand(intake, shooter, leds),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(
                    OUR_BOTTOM_FENDER,
                    innerWaypoints(OUR_CARGO_5, OUR_TERMINAL_CARGO),
                    OUR_BOTTOM_FENDER,
                    FORWARD())),
            intake.intakeBallCommand()),
        new ShootBallsCommand(intake, shooter, leds));
  }
}
