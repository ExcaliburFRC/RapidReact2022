package io.excaliburfrc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import io.excaliburfrc.lib.RepeatingCommand;
import io.excaliburfrc.robot.subsystems.LEDs.LedMode;

public class Superstructure {
  public final Intake intake = new Intake();
  public final Shooter shooter = new Shooter();

  public Command shootBallsCommand(LEDs leds) {
    return new WaitUntilCommand(intake::isEmpty)
        .deadlineWith(
            shooter.accelerateFenderCommand(),
            new RepeatingCommand(
                sequence(new WaitUntilCommand(shooter.atTargetVelocity), intake.pullIntoShooter())),
            leds.setColorCommand(LedMode.PINK));
  }

  public Command intakeBallCommand() {
    return intake
        .openPiston()
        .andThen(intake.pullIntoIntake())
        .andThen(
            new ConditionalCommand(
                // increment ball count; input until upper sensor detects a ball
                intake.pullIntoUpper(),
                // decrement ball count
                // output ball from intake / shooter
                new ConditionalCommand(
                    // outputs the ball from the shooter
                    intake.pullIntoShooter(),
                    // outputs the ball from intake
                    intake.ejectFromIntake(),
                    // decides to output from intake or from shooter
                    intake.upperBallTrigger),
                // decides by ball color
                intake::isOurColor));
  }

  public Command ejectBallCommand() {
    return sequence(intake.ejectFromIntake(), intake.ejectFromUpper());
  }

  public void resetBallCounter() {
    intake.resetBallCounter();
  }
}
