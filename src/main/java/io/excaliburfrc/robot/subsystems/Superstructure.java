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

  public Command teleopShootBallCommand(Drive drive,LEDs leds){
    return new ParallelDeadlineGroup(
            new WaitUntilCommand(intake::isEmpty),
            new ParallelDeadlineGroup(
                    shooter.accelerateFenderCommand(),
                    drive.driveDis(drive.getOdometryPose().getTranslation(), -0.3, 0.6)),
            drive.stop(),
            new RepeatingCommand(
                    sequence(
                            new WaitUntilCommand(new Trigger(shooter::isAtTargetVelocity).debounce(0.2)),
                            intake.pullIntoShooter(Falling(shooter.ballShotTrigger)))),
            sequence(intake.intakeTick()),
            leds.setColorCommand(LedMode.VIOLET));
  }

  private Command sortBallsCommand(){
    // intakes ball
    return intake.pullIntoIntake().andThen(
    // weather the ball is our alliance's
    new ConditionalCommand(
          // weather there is a ball on "upper"
          new ConditionalCommand(
                // true -> do nothing
                new InstantCommand(),
                // false -> pull the ball to "upper"
                intake.pullIntoUpper().until(intake.upperBallTrigger),
                intake.upperBallTrigger
          ),
          // weather there is a ball on "upper"
          new ConditionalCommand(
                // true -> eject from intake
                intake.ejectFromIntake(),
                // false -> pull to upper and then weak shooter so the ball stays in the field
                intake.pullIntoUpper().andThen(intake.pullIntoShooter(intake.upperBallTrigger.negate().debounce(1))).deadlineWith(shooter.ejectLow()),
                intake.upperBallTrigger
          ),
          intake::isOurColor));
  }

//  public Command intakeBallCommand() {
//    return new ConditionalCommand(
//            intake.pullIntoUpper(), new InstantCommand(), intake.intakeBallTrigger)
//        .andThen(intake.openPiston())
//          .andThen(new WaitCommand(0.3))
//        .andThen(intake.pullIntoIntake())
//        .andThen(
//            new ConditionalCommand(
//                // increment ball count; input until upper sensor detects a ball
//                intake.pullIntoUpper(),
//                // decrement ball count
//                // output ball from intake / shooter
//                new ConditionalCommand(
//                    // outputs the ball from the shooter
//                    intake
//                        .pullIntoUpper()
//                        .andThen(intake.pullIntoShooter(Falling(intake.upperBallTrigger)))
//                        .deadlineWith(shooter.ejectLow()),
//                    // outputs the ball from intake
//                    intake.ejectFromIntake(),
//                    // decides to output from intake or from shooter
//                    intake.upperBallTrigger.negate()),
//                // decides by ball color
//                intake::isOurColor));
//  }

  public Command intakeBallsCommand(){
    return new ParallelDeadlineGroup(
          // sets the command to stop when the intake has two balls in it
          new WaitUntilCommand(()-> intake.intakeFull() || intake.getBallCount() == 2),
                // opens the intake's pistons
                intake.openPiston()
                      .andThen(
                // runs the sorting command until command is stopped
                new RepeatingCommand(sortBallsCommand())));
  }

  public Command ejectBallCommand() {
    return intake.rawEject().withTimeout(1).andThen(()-> intake.resetBallCounter(0));
  }

  public Command resetBallCounterCommand(int n) {
    return new InstantCommand(() -> intake.resetBallCounter(n));
  }
}
