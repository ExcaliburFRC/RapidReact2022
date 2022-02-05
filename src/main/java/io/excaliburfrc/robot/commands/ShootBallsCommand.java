package io.excaliburfrc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.excaliburfrc.lib.RepeatingCommand;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.Shooter;

public class ShootBallsCommand extends ParallelDeadlineGroup {
  /**
   * @param trigger an additional trigger, will release a ball only when this (and the velocity
   *     check) are reached.
   */
  public ShootBallsCommand(Intake intake, Shooter shooter, Trigger trigger) {
    super(
        // FIXME: handle mechanism failures? (ie stuck balls, etc)
        new WaitUntilCommand(intake::isEmpty),
        shooter.accelerateFenderCommand(),
        new RepeatingCommand(
            new SequentialCommandGroup(
                new WaitUntilCommand(new Trigger(shooter::isAtTargetVelocity).and(trigger)),
                intake.shootBallCommand())));
  }

  public ShootBallsCommand(Intake intake, Shooter shooter) {
    this(intake, shooter, new Trigger(() -> true));
  }
}
