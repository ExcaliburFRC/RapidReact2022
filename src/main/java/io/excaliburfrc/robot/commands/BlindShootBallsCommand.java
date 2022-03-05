package io.excaliburfrc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.excaliburfrc.lib.RepeatingCommand;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.LEDs.LedMode;
import io.excaliburfrc.robot.subsystems.Shooter;

public class BlindShootBallsCommand extends ParallelCommandGroup {
  /**
   * @param trigger an additional trigger, will release a ball only when this (and the velocity
   *     check) are reached.
   */
  public BlindShootBallsCommand(Intake intake, Shooter shooter, LEDs leds, Trigger trigger) {
    super(
        shooter.accelerateFenderCommand(),
        new RepeatingCommand(
            new SequentialCommandGroup(
                new WaitUntilCommand(new Trigger(shooter::isAtTargetVelocity).and(trigger)),
                intake.blindShootBallCommand())),
        leds.setColorCommand(LedMode.PINK));
  }

  public BlindShootBallsCommand(Intake intake, Shooter shooter, LEDs leds) {
    this(intake, shooter, leds, new Trigger(() -> true));
  }
}
