package io.excaliburfrc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.ShootBallsCommand;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Shooter;

public class NoRamsete extends SequentialCommandGroup {
  public NoRamsete(Drive drive, Intake intake, Shooter shooter, LEDs leds) {
    super(
        drive.resetOdometryCommand(Trajectories.OUR_TOP_FENDER),
        new ShootBallsCommand(intake, shooter, leds),
        drive.arcadeDriveCommand(() -> -0.4, () -> 0.0).withTimeout(5),
        drive.arcadeDriveCommand(() -> 0, () -> 0));
  }
}
