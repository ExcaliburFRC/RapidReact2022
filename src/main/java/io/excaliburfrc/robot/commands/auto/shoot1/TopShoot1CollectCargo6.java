package io.excaliburfrc.robot.commands.auto.shoot1;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;
import static io.excaliburfrc.robot.commands.auto.Trajectories.FORWARD;

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
 * <p>1. shoot 1
 *
 * <p>2. reverse into THEIR_CARGO_6
 *
 * <p>3. drive and collect OUR_6
 */
public class TopShoot1CollectCargo6 extends SequentialCommandGroup {
  private static final Pose2d reversePoint = new Pose2d(THEIR_CARGO_6, degrees(159.0));

  public TopShoot1CollectCargo6(Drive drive, Shooter shooter, Intake intake, LEDs leds) {
    super(
        drive.resetOdometryCommand(OUR_TOP_FENDER),
        new ShootBallsCommand(intake, shooter, leds),
        drive.followTrajectoryCommand(
            generateTrajectory(OUR_TOP_FENDER, innerWaypoints(), reversePoint, REVERSE())),
        parallel(
            drive.followTrajectoryCommand(
                generateTrajectory(
                    reversePoint, innerWaypoints(OUR_CARGO_6), OUR_TOP_FENDER, FORWARD())),
            intake.intakeBallCommand()));
  }
}
