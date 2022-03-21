package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import io.excaliburfrc.robot.commands.auto.noramsete.NoRamseteBottomFender;
import io.excaliburfrc.robot.commands.auto.noramsete.NoRamseteTopFender;
import io.excaliburfrc.robot.commands.auto.twoBalls.Cargo4Bottom;
import io.excaliburfrc.robot.commands.auto.twoBalls.Cargo5Bottom;
import io.excaliburfrc.robot.commands.auto.twoBalls.Cargo6Bottom;
import io.excaliburfrc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final PS4Controller driveJoystick = new PS4Controller(0);
  private final PS4Controller armJoystick = new PS4Controller(1);
  // The robot's subsystems and commands are defined here...
  private final Superstructure superstructure = new Superstructure();
  private final Climber climber = new Climber();
  private final Drive drive = new Drive();
  private final LEDs leds = new LEDs();

  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    //    chooser.addOption(
    //        "4Ball", new FourBallAuto(drive, superstructure, leds));
    chooser.addOption(
        "TopOut",
        new NoRamseteTopFender(drive, superstructure.intake, superstructure.shooter, leds));
    chooser.addOption(
        "BottomOut",
        new NoRamseteBottomFender(drive, superstructure.intake, superstructure.shooter, leds));
    chooser.addOption("2BallsCargo4", new Cargo4Bottom(drive, superstructure, leds));
    chooser.addOption("2BallsCargo5", new Cargo5Bottom(drive, superstructure, leds));
    chooser.addOption("2BallsCargo6", new Cargo6Bottom(drive, superstructure, leds));
    SmartDashboard.putData("Autos", chooser);
  }

  void configureButtonBindings() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    drive.setDefaultCommand(
        drive.arcadeDriveCommand(
            () -> -driveJoystick.getLeftY(), driveJoystick::getRightX, driveJoystick::getR1Button));

    leds.setDefaultCommand(leds.setColorCommand(leds.getAlliance()));

    new Button(armJoystick::getR2Button).toggleWhenPressed(superstructure.shootBallsCommand(leds));

    // when intake is required
    new Button(() -> CommandScheduler.getInstance().requiring(superstructure.intake) != null)
        .whenReleased(superstructure.intake.closePiston());

    new Button(armJoystick::getL2Button).toggleWhenPressed(superstructure.intakeBallCommand());

    new Button(armJoystick::getL1Button).toggleWhenPressed(superstructure.ejectBallCommand());

    climber
        .climberManualCommand(
            armJoystick::getTriangleButton,
            armJoystick::getCrossButton,
            () -> armJoystick.getPOV() == 0,
            () -> armJoystick.getPOV() == 180,
            () -> armJoystick.getPOV() == 90,
            () -> armJoystick.getPOV() == 270)
        .schedule();

    new Button(armJoystick::getR1Button).whileActiveOnce(climber.disableSoftLimits());

    new Button(driveJoystick::getShareButton)
        .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    new POVButton(driveJoystick, 0).whenPressed(superstructure.shooter.incrementTarget(1));
    new POVButton(driveJoystick, 180).whenPressed(superstructure.shooter.incrementTarget(-1));

    new Button(driveJoystick::getOptionsButton)
        .toggleWhenPressed(superstructure.intake.allowCommand());

    DriverStation.reportWarning("Buttons!", false);
  }

  void manualButton() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    drive.setDefaultCommand(
        drive.arcadeDriveCommand(
            () -> -driveJoystick.getLeftY(), driveJoystick::getRightX, driveJoystick::getR1Button));

    climber
        .climberManualCommand(
            armJoystick::getTriangleButton,
            armJoystick::getCrossButton,
            () -> armJoystick.getPOV() == 0,
            () -> armJoystick.getPOV() == 180,
            () -> armJoystick.getPOV() == 90,
            () -> armJoystick.getPOV() == 270)
        .schedule();

    new Button(armJoystick::getR1Button).whileActiveOnce(climber.disableSoftLimits());

    new Button(driveJoystick::getShareButton)
        .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    DriverStation.reportWarning("Manual!", false);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
