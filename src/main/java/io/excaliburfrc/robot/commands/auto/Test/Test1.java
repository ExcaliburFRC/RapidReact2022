package io.excaliburfrc.robot.commands.auto.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.auto.Trajectories;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

public class Test1 extends SequentialCommandGroup {
//  static final Pose2d start = new Pose2d(OUR_CARGO_4, Rotation2d.fromDegrees(90));
  static final Pose2d start = new Pose2d(6.6, 1.65, Rotation2d.fromDegrees(0));
  static final Pose2d end = new Pose2d(5.064, 1.910, Rotation2d.fromDegrees(180));

  public Test1(Drive drive, LEDs leds, Superstructure superstructure) {
    super(
          drive.resetOdometryCommand(start)
    );
  }
}
//I first shoot two balls and then I go a little bit back to stop1 so I can take a turn
// and then go to ball 6 and the go back to shoot one more ball
