package io.excaliburfrc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.Vision;
import java.util.Arrays;

public class AutomaticIntakeCommand extends CommandBase {
  private final Drive drive;
  private final Intake intake;
  private final Vision vision;
  private Trajectory trajectory;

  public AutomaticIntakeCommand(Drive drive, Intake intake, Vision vision) {
    this.drive = drive;
    this.intake = intake;
    this.vision = vision;

    andThen(drive.followTrajectoryCommand(trajectory)).raceWith(intake.intakeBallCommand());
  }

  @Override
  public void initialize() {
    var currentPose = drive.getPose();
    trajectory =
        TrajectoryGenerator.generateTrajectory(
            Arrays.asList(currentPose, currentPose.plus(vision.getTransform())),
            drive.trajectoryConfig);
  }
}
