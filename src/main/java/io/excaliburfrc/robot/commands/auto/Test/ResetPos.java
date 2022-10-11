package io.excaliburfrc.robot.commands.auto.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;
import io.excaliburfrc.robot.commands.auto.Trajectories;

public class ResetPos extends SequentialCommandGroup {
//  static final Pose2d start = new Pose2d(OUR_CARGO_4, Rotation2d.fromDegrees(90));
  static final Pose2d start = new Pose2d(Trajectories.OUR_CARGO_4.getX(), 3.192, Rotation2d.fromDegrees(90));

  public ResetPos(Drive drive, LEDs leds, Superstructure superstructure) {
    super(
          drive.resetOdometryCommand(start)
    );
  }
}
//I first shoot two balls and then I go a little bit back to stop1 so I can take a turn
// and then go to ball 6 and the go back to shoot one more ball
