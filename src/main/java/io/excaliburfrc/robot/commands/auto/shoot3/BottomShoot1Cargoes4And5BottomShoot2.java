package io.excaliburfrc.robot.commands.auto.shoot3;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.ShootBallsCommand;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Shooter;

/**
 * 0. OUR_BOTTOM_FENDER
 *
 * <p>2. shoot x1
 *
 * <p>3. go to OUR_CARGO_4
 *
 * <p>4. go to OUR_CARGO_5
 *
 * <p>5. go to OUR_BOTTOM_FENDER
 *
 * <p>6. shoot x2
 */
public class BottomShoot1Cargoes4And5BottomShoot2 extends SequentialCommandGroup {

  public BottomShoot1Cargoes4And5BottomShoot2(Drive drive, Shooter shooter, Intake intake, LEDs leds) {
    super(
        drive.resetOdometryCommand(OUR_BOTTOM_FENDER),
        new ShootBallsCommand(intake, shooter, leds),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(
                    OUR_BOTTOM_FENDER,
                    innerWaypoints(OUR_CARGO_4, OUR_CARGO_5),
                    OUR_BOTTOM_FENDER,
                    FORWARD())),
            intake.intakeBallCommand().andThen(intake.intakeBallCommand())),
        new ShootBallsCommand(intake, shooter, leds));
  }
}
