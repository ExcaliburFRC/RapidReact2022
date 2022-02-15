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
 * <p>1. shoot 1
 *
 * <p>2.Drive and collect OUR_5
 */
public class BottomShoot1CollectCargo5 extends SequentialCommandGroup {
  public BottomShoot1CollectCargo5(Drive drive, Shooter shooter, Intake intake, LEDs leds) {
    super(
        drive.resetOdometryCommand(OUR_BOTTOM_FENDER),
        new ShootBallsCommand(intake, shooter, leds),
        parallel(
            drive.followTrajectoryCommand(
                generateTrajectory(
                    OUR_BOTTOM_FENDER,
                    innerWaypoints(),
                    new Pose2d(OUR_CARGO_5, null) // FIXME
                    ,
                    FORWARD())),
            intake.intakeBallCommand()));
  }
}
