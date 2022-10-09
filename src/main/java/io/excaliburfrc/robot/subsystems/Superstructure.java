package io.excaliburfrc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.sequence;
import static io.excaliburfrc.lib.TriggerUtils.Falling;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.excaliburfrc.lib.RepeatingCommand;
import io.excaliburfrc.robot.subsystems.LEDs.LedMode;

public class Superstructure {
  public final Intake intake = new Intake();
  public final Shooter shooter = new Shooter();

  public Command shootBallsCommand(LEDs leds) {
//    return drive.arcadeDriveCommand(()-> 0, ()-> -0.3)
//          .until(()-> drive.getDistanceFromHub() >= 0.61).andThen(
          return new ParallelDeadlineGroup(
        new WaitUntilCommand(intake::isEmpty),
        shooter.accelerateFenderCommand(),
        new RepeatingCommand(
            sequence(
                new WaitUntilCommand(new Trigger(shooter::isAtTargetVelocity).debounce(0.2)),
                intake.pullIntoShooter(Falling(shooter.ballShotTrigger)))),
        sequence(intake.intakeTick()),
        leds.setColorCommand(LedMode.VIOLET));
  }

  public Command intakeBallCommand() {
    return new ConditionalCommand(
            intake.pullIntoUpper(), new InstantCommand(), intake.intakeBallTrigger)
        .andThen(intake.openPiston())
          .andThen(new WaitCommand(0.3))
        .andThen(intake.pullIntoIntake())
        .andThen(
            new ConditionalCommand(
                // increment ball count; input until upper sensor detects a ball
                intake.pullIntoUpper(),
                // decrement ball count
                // output ball from intake / shooter
                new ConditionalCommand(
                    // outputs the ball from the shooter
                    intake
                        .pullIntoUpper()
                        .andThen(intake.pullIntoShooter(Falling(intake.upperBallTrigger)))
                        .deadlineWith(shooter.ejectLow()),
                    // outputs the ball from intake
                    intake.ejectFromIntake(),
                    // decides to output from intake or from shooter
                    intake.upperBallTrigger.negate()),
                // decides by ball color
                intake::isOurColor),
            intake.closePiston());
  }

  public Command ejectBallCommand() {
    return intake.rawEject().withTimeout(0.8);
  }

  public Command resetBallCounterCommand(int n) {
    return new InstantCommand(() -> intake.resetBallCounter(n));
  }
}
