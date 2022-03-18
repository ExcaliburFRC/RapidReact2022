package io.excaliburfrc.robot.commands.auto.noramsete;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.BlindShootBallsCommand;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Shooter;

public class NoRamseteTopFender extends SequentialCommandGroup {
  public NoRamseteTopFender(Drive drive, Intake intake, Shooter shooter, LEDs leds) {
    super(
        drive.resetOdometryCommand(new Pose2d(7.5, 4.45, fromDegrees(159.0))),
        new BlindShootBallsCommand(intake, shooter, leds).withTimeout(3),
        drive.arcadeDriveCommand(() -> -0.4, () -> 0.0).withTimeout(4),
        drive.arcadeDriveCommand(() -> 0.0, () -> 0.0));
  }
}
