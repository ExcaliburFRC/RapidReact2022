package io.excaliburfrc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Shooter;

public class NoRamsete extends SequentialCommandGroup {
    public NoRamsete(Drive drive, Intake intake, Shooter shooter , LEDs leds) {
        super(
                drive.resetOdometryCommand(new Pose2d(7.5, 4.45, Rotation2d.fromDegrees(159.0))),
                new ShootBallsCommand(intake, shooter, leds),
                drive.arcadeDriveCommand(() -> -0.4, () -> 0.0, () -> false).withTimeout(5),
                drive.arcadeDriveCommand(() -> 0, () -> 0, () -> false));
    }
}
