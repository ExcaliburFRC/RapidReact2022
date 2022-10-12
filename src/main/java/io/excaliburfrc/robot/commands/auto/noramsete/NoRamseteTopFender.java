package io.excaliburfrc.robot.commands.auto.noramsete;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.BlindShootBallsCommand;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

public class NoRamseteTopFender extends SequentialCommandGroup {
  public NoRamseteTopFender(Drive drive, Superstructure superstructure, LEDs leds) {
    super(
        drive.resetOdometryCommand(new Pose2d(7.5, 4.45, fromDegrees(159.0))),
        new BlindShootBallsCommand(superstructure.intake, superstructure.shooter, leds).withTimeout(4),
        drive.arcadeDriveCommand(() -> -0.4, () -> 0.0).withTimeout(4),
        drive.arcadeDriveCommand(() -> 0.0, () -> 0.0));
  }
}
