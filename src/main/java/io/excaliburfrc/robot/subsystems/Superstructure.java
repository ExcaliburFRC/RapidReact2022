package io.excaliburfrc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.sequence;

import edu.wpi.first.wpilibj2.command.*;
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
            leds.setColorCommand(LedMode.GREEN));
  }

  public Command intakeBallCommand() {
    return new ConditionalCommand(
            intake.pullIntoUpper(), new InstantCommand(), intake.intakeBallTrigger)
        .andThen(intake.openPiston())
        .andThen(intake.pullIntoIntake())
        .andThen(
            new ConditionalCommand(
                // increment ball count; input until upper sensor detects a ball
                intake.pullIntoUpper(),
                // decrement ball count
                // output ball from intake / shooter
                new ConditionalCommand(
                    // outputs the ball from the shooter
                    intake.pullIntoUpper().andThen(intake.pullIntoShooter()).alongWith(shooter.ejectLow()),
                    // outputs the ball from intake
                    intake.ejectFromIntake(),
                    // decides to output from intake or from shooter
                    intake.upperBallTrigger.negate()),
                // decides by ball color
                intake::isOurColor));
  }

//  public Command ejectBallCommand() {
//    return sequence(intake.closePiston(), intake.ejectFromIntake(), intake.ejectFromUpper(), intake.ejectFromIntake())
//        .until(intake::isEmpty);
//  }

  public Command ejectBallCommand(){
    return intake.rawEject();
  }

  public Command resetBallCounterCommand() {
    return new InstantCommand(intake::resetBallCounter);
  }
}
