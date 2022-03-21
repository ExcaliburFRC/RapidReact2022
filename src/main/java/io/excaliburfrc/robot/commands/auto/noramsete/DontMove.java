package io.excaliburfrc.robot.commands.auto.noramsete;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.BlindShootBallsCommand;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Shooter;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

public class DontMove extends SequentialCommandGroup {
  public DontMove(Drive drive, Intake intake, Shooter shooter, LEDs leds) {
    super(
        drive.resetOdometryCommand(new Pose2d(8, 3.3, fromDegrees(204.0))),
        new BlindShootBallsCommand(intake, shooter, leds).withTimeout(3));
  }
}
