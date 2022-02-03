package io.excaliburfrc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.Shooter;

public class ShootBallsCommand extends ParallelDeadlineGroup {
  public ShootBallsCommand(Intake intake, Shooter shooter) {
    super(
        // FIXME: handle mechanism failures? (ie stuck balls, etc)
        new WaitUntilCommand(intake::isEmpty),
        shooter.accelerateFenderCommand(),
        // motor

        // TODO: check perpetuating. for now, duplicating is fine
        new SequentialCommandGroup(
            new WaitUntilCommand(shooter::isAtTargetVelocity),
            intake.shootBallCommand(),
            new WaitUntilCommand(shooter::isAtTargetVelocity),
            intake.shootBallCommand()));
  }
}
