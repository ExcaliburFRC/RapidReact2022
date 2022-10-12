package io.excaliburfrc.robot.commands.auto.noramsete;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.BlindShootBallsCommand;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

public class NoRamseteBottomFender extends SequentialCommandGroup {
  public NoRamseteBottomFender(Drive drive, Superstructure superstracture, LEDs leds) {
    super(
        drive.resetOdometryCommand(new Pose2d(8, 3.3, fromDegrees(204.0))),
        new BlindShootBallsCommand(superstracture.intake, superstracture.shooter, leds).withTimeout(3),
        drive.arcadeDriveCommand(() -> -0.4, () -> 0.0).withTimeout(4),
        drive.arcadeDriveCommand(() -> 0.0, () -> 0.0));
  }
}
