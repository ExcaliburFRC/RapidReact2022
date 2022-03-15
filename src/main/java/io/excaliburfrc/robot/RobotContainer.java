package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import io.excaliburfrc.robot.commands.NoRamseteBottomFender;
import io.excaliburfrc.robot.commands.NoRamseteTopFender;
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
    chooser.addOption(
        "Top", new NoRamseteTopFender(drive, superstructure.intake, superstructure.shooter, leds));
    chooser.addOption(
        "Bottom",
        new NoRamseteBottomFender(drive, superstructure.intake, superstructure.shooter, leds));
    SmartDashboard.putData("Autos", chooser);
  }

  void configureButtonBindings() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    drive.setDefaultCommand(
        drive.arcadeDriveCommand(
            () -> -driveJoystick.getLeftY(), driveJoystick::getRightX, driveJoystick::getR1Button));

    leds.setDefaultCommand(leds.setColorCommand(leds.getAlliance()));

    new Button(armJoystick::getR2Button).whenPressed(superstructure.shootBallsCommand(leds));

    new Button(armJoystick::getL2Button)
        .whenReleased(superstructure.intake.closePiston())
        .whenPressed(superstructure.intakeBallCommand());

    new Button(armJoystick::getL1Button).whileActiveOnce(superstructure.ejectBallCommand());

    climber
        .climberManualCommand(
            armJoystick::getTriangleButton,
              armJoystick::getCrossButton,
              () -> armJoystick.getPOV() == 0,
              () -> armJoystick.getPOV() == 180,
              () -> armJoystick.getPOV() == 90,
            () -> armJoystick.getPOV() == 270)
        .schedule();

    new Button(armJoystick::getL1Button).whileActiveOnce(climber.disableSoftLimits());

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

    superstructure.intake.setDefaultCommand(
        superstructure.intake.manualCommand(
            () -> armJoystick.getRawButton(2),
            () -> armJoystick.getRawButton(4),
            () -> armJoystick.getRawButton(5),
            () -> armJoystick.getRawButton(6),
            () -> armJoystick.getRawButtonPressed(8)));

    drive.setDefaultCommand(
        drive.arcadeDriveCommand(
            () -> -driveJoystick.getLeftY(), driveJoystick::getRightX, driveJoystick::getR1Button));
    new JoystickButton(armJoystick, 1).whenHeld(superstructure.shooter.accelerateFenderCommand());
    new JoystickButton(armJoystick, 9).whenHeld(superstructure.shooter.manualCommand());

    new JoystickButton(armJoystick, 12).whenHeld(superstructure.shootBallsCommand(leds));

    climber
        .climberTuneCommand(
            () -> driveJoystick.getPOV() == 0,
            () -> driveJoystick.getPOV() == 180,
            driveJoystick::getTriangleButton,
            driveJoystick::getCrossButton,
            () -> driveJoystick.getPOV() == 90,
            () -> driveJoystick.getPOV() == 270)
        .schedule();

    new Button(driveJoystick::getShareButton)
        .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    new Button(driveJoystick::getL1Button).whileActiveOnce(climber.disableSoftLimits());

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
