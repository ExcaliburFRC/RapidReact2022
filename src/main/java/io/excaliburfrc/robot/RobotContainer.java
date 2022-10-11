package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import io.excaliburfrc.robot.commands.auto.Test.Test1;
import io.excaliburfrc.robot.commands.auto.oneBall.ballFive;
import io.excaliburfrc.robot.commands.auto.oneBall.ballFour;
import io.excaliburfrc.robot.commands.auto.twoBalls.FiveTerminal;
import io.excaliburfrc.robot.commands.auto.twoBalls.fiveFour;
import io.excaliburfrc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController driveJoystick = new XboxController(0);
  private final PS4Controller armJoystick = new PS4Controller(1);
  // The robot's subsystems and commands are defined here...
  private final Superstructure superstructure = new Superstructure();
  private final Climber climber = new Climber();
  private final Drive drive = new Drive();
  private final LEDs leds = new LEDs();

  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  private final SendableChooser<Command> chooser = new SendableChooser<>();
  public final SendableChooser<Integer> initialBallCounter = new SendableChooser<>();

  public RobotContainer() {
        chooser.addOption(
            "fiveFour", new fiveFour(drive, leds, superstructure));
        chooser.addOption(
            "ballFive", new ballFive(drive, leds, superstructure));
        chooser.addOption(
            "ballFour", new ballFour(drive, leds, superstructure));
        chooser.addOption(
            "FiveTerminal", new FiveTerminal(drive, leds, superstructure));
        chooser.addOption(
            "test", new Test1(drive, leds, superstructure));

    SmartDashboard.putData("Autos", chooser);

    initialBallCounter.addOption("1", 1);
    initialBallCounter.addOption("2", 2);

    SmartDashboard.putData("initial balls", initialBallCounter);
  }

  void configureButtonBindings() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    drive.setDefaultCommand(
        drive.arcadeDriveCommand(
            () -> -driveJoystick.getLeftY(), driveJoystick::getRightX));

    leds.setDefaultCommand(leds.setColorCommand(leds.getAlliance()));

    new Button(armJoystick::getR2Button)
        .toggleWhenPressed(superstructure.shootBallsCommand(leds));

    //    new Button(driveJoystick::getR2Button)
    //          .toggleWhenPressed(superstructure.shooter.manualCommand(()-> 0.5));

    //    new Button(driveJoystick::getCircleButton)
    //        .toggleWhenPressed(
    //            drive.rotateToHub().deadlineWith(leds.setColorCommand(LEDs.LedMode.YELLOW)));

    new Button(driveJoystick::getTouchpadPressed)
            .toggleWhenPressed(drive.toggleSpeedCommand());

    // when intake is required
    new Button(() -> CommandScheduler.getInstance().requiring(superstructure.intake) != null)
        .whenReleased(superstructure.intake.closePiston());

    new Button(armJoystick::getL2Button).toggleWhenPressed(superstructure.intakeBallCommand());

    new Button(armJoystick::getL1Button).toggleWhenPressed(superstructure.ejectBallCommand());

//    new Button(()-> driveJoystick.getTriangleButton() && driveJoystick.getPOV() == 0)
//          .toggleWhenPressed(
//                new StartEndCommand(
//                      ()-> drive.setMaxOutput(0.2),
//                      ()-> drive.setMaxOutput(1)));

    climber
        .climberManualCommand(
            () -> armJoystick.getPOV() == 0,
            () -> armJoystick.getPOV() == 180,
            armJoystick::getTriangleButton,
            armJoystick::getCrossButton,
            () -> armJoystick.getPOV() == 90,
            () -> armJoystick.getPOV() == 270,
            armJoystick::getR1Button)
        .schedule();

    new Button(driveJoystick::getXButtonPressed)
        .toggleWhenPressed(
            new ConditionalCommand(
                superstructure.intake.closePiston(),
                superstructure.intake.openPiston(),
                superstructure.intake::isOpen));

    //    new Button(driveJoystick::getR1Button).whileActiveOnce(climber.disableSoftLimits());

    new Button(()-> driveJoystick.getRawButtonPressed(7))
        .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    new POVButton(driveJoystick, 0).whenPressed(superstructure.shooter.incrementTarget(1));
    new POVButton(driveJoystick, 180).whenPressed(superstructure.shooter.incrementTarget(-1));

    new Button(()-> driveJoystick.getRawButtonPressed(8))
        .toggleWhenPressed(superstructure.intake.allowCommand());

    new Button(()-> driveJoystick.getRawButton(12))
          .whenPressed(
                new PrintCommand(drive.getOdometryPose().toString()));

    DriverStation.reportWarning("Buttons!", false);
  }

  void manualButton() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    drive.setDefaultCommand(
          drive.arcadeDriveCommand(
                () -> -driveJoystick.getLeftY(), driveJoystick::getRightX));

    leds.setDefaultCommand(leds.setColorCommand(leds.getAlliance()));

    new Button(()-> driveJoystick.getRightTriggerAxis() > 0.1)
          .toggleWhenPressed(superstructure.shootBallsCommand(leds));

    // when intake is required
    new Button(() -> CommandScheduler.getInstance().requiring(superstructure.intake) != null)
          .whenReleased(superstructure.intake.closePiston());

    new Button(()-> driveJoystick.getLeftTriggerAxis() > 0.1).toggleWhenPressed(superstructure.intakeBallCommand());

    new Button(driveJoystick::getLeftBumperPressed).toggleWhenPressed(superstructure.ejectBallCommand());

    climber
          .climberManualCommand(
                () -> driveJoystick.getPOV() == 0,
                () -> driveJoystick.getPOV() == 180,
                driveJoystick::getYButton,
                driveJoystick::getAButton,
                () -> driveJoystick.getPOV() == 90,
                () -> driveJoystick.getPOV() == 270,
                driveJoystick::getRightBumper)

          .schedule();

    new Button(driveJoystick::getXButtonPressed)
          .toggleWhenPressed(
                new ConditionalCommand(
                      superstructure.intake.closePiston(),
                      superstructure.intake.openPiston(),
                      superstructure.intake::isOpen));

    //    new Button(driveJoystick::getR1Button).whileActiveOnce(climber.disableSoftLimits());

    new Button(()-> driveJoystick.getRawButtonPressed(7))
          .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    new Button(()-> driveJoystick.getRawButtonPressed(8))
          .toggleWhenPressed(superstructure.intake.allowCommand());

    new Button(()-> driveJoystick.getRightBumperPressed())
          .whenPressed(
                new PrintCommand(drive.getOdometryPose().toString()));

    DriverStation.reportWarning("Manual!", false);
  }

  public void setBallCount(int n){
    superstructure.intake.resetBallCounter(n);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
  public int getInitBalls() {
    return initialBallCounter.getSelected();
  }
}
