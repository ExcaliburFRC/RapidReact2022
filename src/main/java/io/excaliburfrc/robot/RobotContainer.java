package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import io.excaliburfrc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final PS4Controller driveJoystick = new PS4Controller(0);
  private final Joystick armJoystick = new Joystick(1);
  // The robot's subsystems and commands are defined here...
  private final Climber climber = new Climber();
  private final Drive drive = new Drive();
  private final LEDs leds = new LEDs();

  private final Superstructure superstructure = new Superstructure();

  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public RobotContainer() {}

  void configureButtonBindings() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    drive.setDefaultCommand(
        drive.arcadeDriveCommand(
            () -> -driveJoystick.getLeftY(), driveJoystick::getRightX, driveJoystick::getR1Button));

    leds.setDefaultCommand(leds.setColorCommand(leds.getAlliance()));

    new JoystickButton(armJoystick, 1).whileActiveOnce(superstructure.shootBallsCommand(leds));

    new JoystickButton(armJoystick, 2)
        .whileActiveContinuous(superstructure.intakeBallCommand())
        .whenInactive(superstructure.closePistonCommand());

    new JoystickButton(armJoystick, 4).whileHeld(superstructure.ejectFromIntake());

    //    var stepButton = new Button(() -> armJoystick.getRawButton(3));
    //    new POVButton(driveJoystick, POV_UP)
    //        .whenPressed(
    //            climber.climbSeries(
    //                stepButton, stepButton, stepButton, stepButton, stepButton, stepButton));

    climber
        .climberManualCommand(
            () -> driveJoystick.getPOV() == 0,
            () -> driveJoystick.getPOV() == 180,
            driveJoystick::getTriangleButton,
            driveJoystick::getCrossButton,
            () -> driveJoystick.getPOV() == 90,
            () -> driveJoystick.getPOV() == 270)
        .schedule();

    new Button(driveJoystick::getL1Button).whileActiveOnce(climber.disableSoftLimits());

    new Button(driveJoystick::getShareButton)
        .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    new POVButton(armJoystick, 0).whenPressed(superstructure.incrementTarget(1));
    new POVButton(armJoystick, 180).whenPressed(superstructure.incrementTarget(-1));

    new Button(driveJoystick::getOptionsButton).toggleWhenPressed(superstructure.allowCommand());

    DriverStation.reportWarning("Buttons!", false);
  }
}
