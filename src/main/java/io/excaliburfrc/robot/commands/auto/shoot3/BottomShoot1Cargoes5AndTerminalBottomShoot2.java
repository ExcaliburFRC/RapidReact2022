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
 * <p>1. shoot 1
 *
 * <p>2. go to OUR_CARGO_5
 *
 * <p>3. go to OUR_TERMINAL_CARGO
 *
 * <p>4. go to OUR_BOTTOM_FENDER
 *
 * <p>5. shoot 2
 */
public class BottomShoot1Cargoes5AndTerminalBottomShoot2 extends SequentialCommandGroup {

  public BottomShoot1Cargoes5AndTerminalBottomShoot2(Drive drive, Shooter shooter, Intake intake, LEDs leds) {
    super(
        drive.resetOdometryCommand(OUR_BOTTOM_FENDER),
        new ShootBallsCommand(intake, shooter, leds),
        deadline(
            drive.followTrajectoryCommand(
                generateTrajectory(
                    OUR_BOTTOM_FENDER,
                    innerWaypoints(OUR_CARGO_5, OUR_TERMINAL_CARGO),
                    OUR_BOTTOM_FENDER,
                    FORWARD())),
            intake.intakeBallCommand().andThen(intake.intakeBallCommand())),
        new ShootBallsCommand(intake, shooter, leds));
  }
}
