package io.excaliburfrc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.lib.RepeatingCommand;

public class Superstructure extends SubsystemBase {
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();

  public Command shootBallsCommand(LEDs leds) {
    return new ParallelDeadlineGroup(
        new WaitUntilCommand(intake::isEmpty),
        shooter.accelerateFenderCommand(),
        new RepeatingCommand(
            new SequentialCommandGroup(
                new WaitUntilCommand(shooter::isAtTargetVelocity), intake.pullIntoShooter())),
        leds.setColorCommand(LEDs.LedMode.PINK));
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
                    intake.ejectFromShooter(),
                    // outputs the ball from intake
                    intake.ejectFromIntake(),
                    // decides to output from intake or from shooter
                    intake.upperBallTrigger),
                // decides by ball color
                intake::isOurColor));
  }
}
