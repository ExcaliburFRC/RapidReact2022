package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import io.excaliburfrc.robot.commands.auto.noramsete.NoRamseteBottomFender;
import io.excaliburfrc.robot.commands.auto.noramsete.NoRamseteTopFender;
import io.excaliburfrc.robot.commands.auto.oneBall.BallFive;
import io.excaliburfrc.robot.commands.auto.oneBall.BallFour;
import io.excaliburfrc.robot.commands.auto.twoBalls.TwoBalls_FiveTerminal;
import io.excaliburfrc.robot.commands.auto.twoBalls.TwoBalls_FiveFour;
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
          "topFenderSimple", new NoRamseteTopFender(drive, superstructure, leds));
    chooser.addOption(
          "bottomFenderSimple", new NoRamseteBottomFender(drive, superstructure, leds));
    chooser.addOption(
            "fiveFour", new TwoBalls_FiveFour(drive, leds, superstructure));
    chooser.addOption(
          "fiveTerminal", new TwoBalls_FiveTerminal(drive, leds, superstructure));
    chooser.addOption(
            "ballFive", new BallFive(drive, leds, superstructure));
    chooser.addOption(
            "ballFour", new BallFour(drive, leds, superstructure));
//        chooser.addOption(
//            "ballSix", new ballSix(drive, leds, superstructure));
//        chooser.addOption(
//            "resetPos", new ResetPos(drive, leds, superstructure));

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

    // when intake is required
    new Button(() -> CommandScheduler.getInstance().requiring(superstructure.intake) != null)
        .whenReleased(superstructure.intake.closePiston());

    new Button(armJoystick::getL2Button).toggleWhenPressed(superstructure.intakeBallCommand());

    new Button(armJoystick::getL1Button).toggleWhenPressed(superstructure.ejectBallCommand());

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

    new Button(()-> driveJoystick.getRawButtonPressed(7))
        .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    new POVButton(driveJoystick, 0).whenPressed(superstructure.shooter.incrementTarget(1));
    new POVButton(driveJoystick, 180).whenPressed(superstructure.shooter.incrementTarget(-1));

    new Button(()-> driveJoystick.getRawButtonPressed(8))
        .toggleWhenPressed(superstructure.intake.allowCommand());

//    new Button(()-> driveJoystick.getRightStickButton())
//          .whenPressed(
//                new PrintCommand(drive.getOdometryPose().toString()));

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

    new Button(()-> driveJoystick.getRawButtonPressed(7))
          .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    new Button(()-> driveJoystick.getRawButtonPressed(8))
          .toggleWhenPressed(superstructure.intake.allowCommand());

    new Button(()-> driveJoystick.getRightStickButton())
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
