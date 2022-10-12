package io.excaliburfrc.robot.commands.auto.noramsete;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.BlindShootBallsCommand;
import io.excaliburfrc.robot.subsystems.*;

public class DontMove extends SequentialCommandGroup {
  public DontMove(Drive drive, Superstructure superstructure, LEDs leds) {
    super(
        drive.resetOdometryCommand(new Pose2d(8, 3.3, fromDegrees(204.0))),
          superstructure.shootBallsCommand(leds)
          );
  }
}
