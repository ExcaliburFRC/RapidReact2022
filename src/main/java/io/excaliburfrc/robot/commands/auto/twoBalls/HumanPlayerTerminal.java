package io.excaliburfrc.robot.commands.auto.twoBalls;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

public class HumanPlayerTerminal extends SequentialCommandGroup {

  private static final Pose2d BOTTOM_SHOOT = new Pose2d(7.644466, 3.025257, degrees(70.000000));
  public static final Pose2d SPIN = new Pose2d(OUR_CARGO_5, Rotation2d.fromDegrees(270));
  private static final Pose2d START = new Pose2d(6.490951, 2.582143, degrees(227.939995));
  public static final Pose2d TERMINAL =
      new Pose2d(new Translation2d(1.3, 1.3), Rotation2d.fromDegrees(223.692797));

  public HumanPlayerTerminal(Drive drive, Superstructure superstructure, LEDs leds) {
    super(
        drive.resetOdometryCommand(START),
        //        drive.resetOdometryCommand(BOTTOM_SHOOT),
        superstructure.resetBallCounterCommand(0),
        //        drive.followTrajectoryCommand(
        //            generateTrajectory(BOTTOM_SHOOT, innerWaypoints(), SPIN, REVERSE())),
        parallel(
            drive.followTrajectoryCommand(
                generateTrajectory(
                    //                    SPIN,
                    START,
                    innerWaypoints(),
                    //                    new Pose2d(OUR_TERMINAL_CARGO,
                    // Rotation2d.fromDegrees(223.692797)),
                    TERMINAL,
                    FORWARD())),
            superstructure.intakeBallCommand()),
        deadline(superstructure.intakeBallCommand(), leds.setColorCommand(LEDs.LedMode.GREEN)),
        drive.followTrajectoryCommand(
            generateTrajectory(TERMINAL, innerWaypoints(), SPIN, REVERSE())
                .concatenate(generateTrajectory(SPIN, innerWaypoints(), BOTTOM_SHOOT, FORWARD()))),
          superstructure.shootBallsCommand(leds));
  }
}
