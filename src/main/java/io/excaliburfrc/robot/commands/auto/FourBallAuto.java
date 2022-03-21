package io.excaliburfrc.robot.commands.auto;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.*;
import io.excaliburfrc.robot.subsystems.LEDs.LedMode;

public class FourBallAuto extends SequentialCommandGroup {

  // Face HUB, back bumper aligned on TARMAC while left corner touches TARMAC
  private static final Pose2d START = new Pose2d(6.8, 5.8, degrees(315));
  public static final Pose2d STOP_1 = new Pose2d(5.39, 7.21, degrees(315));
  public static final Pose2d SHOT_1 = new Pose2d(7, 4.3, degrees(339));
  public static final Pose2d SHOT_1_TWIST = new Pose2d(7, 4.3, degrees(249));
  private static final Pose2d COLLECT_TERMINAL = new Pose2d(OUR_TERMINAL_CARGO, degrees(-130));
  private static final Pose2d COLLECT_TERMINAL_TWIST = new Pose2d(OUR_TERMINAL_CARGO, degrees(50));
  public static final Pose2d SHOT_2 = new Pose2d(7.6, 3.1, degrees(70));

  public FourBallAuto(Drive drive, Superstructure superstructure, LEDs leds) {
    super(
        drive.resetOdometryCommand(START),
        // step I: move backwards out of TARMAC
        drive
            .followTrajectoryCommand(generateTrajectory(START, innerWaypoints(), STOP_1, REVERSE()))
            .alongWith(leds.setColorCommand(LedMode.RED)),
        // step II: move forwards and collect CARGO_6
        parallel(
                drive.followTrajectoryCommand(
                    generateTrajectory(STOP_1, innerWaypoints(OUR_CARGO_6), SHOT_1, FORWARD())),
                superstructure.intakeBallCommand())
            .deadlineWith(leds.setColorCommand(LedMode.GREEN)),
        // step III: shoot (starting, 6) CARGO
        superstructure.shootBallsCommand(leds),
        /// step IV: move to collect TERMINAL_CARGO
        drive.rotateToAngleCommand(SHOT_1_TWIST.getRotation().getDegrees()),
        parallel(
                drive.followTrajectoryCommand(
                    generateTrajectory(
                        SHOT_1_TWIST, innerWaypoints(), COLLECT_TERMINAL, FORWARD())),
                superstructure.intakeBallCommand())
            .deadlineWith(leds.setColorCommand(LedMode.GREEN)),
        // step V: go back and collect CARGO_5
        drive.rotateToAngleCommand(COLLECT_TERMINAL_TWIST.getRotation().getDegrees()),
        parallel(
                drive.followTrajectoryCommand(
                    generateTrajectory(
                        COLLECT_TERMINAL_TWIST, innerWaypoints(OUR_CARGO_5), SHOT_2, FORWARD())),
                superstructure.intakeBallCommand())
            .deadlineWith(leds.setColorCommand(LedMode.GREEN)),
        // step VI: shoot (TERMINAL, 5) CARGO
        superstructure.shootBallsCommand(leds));
  }
}
